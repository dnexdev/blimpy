"""Fake Qwen-Omni realtime server for offline tests of laptop/voice/omni.py (no key, no internet).

  python tools/omni_mock_server.py --port 8765          # then: OMNI_URL=ws://127.0.0.1:8765/v1/realtime OMNI_API_KEY=x python -m laptop.voice.omni

Speaks the OpenAI-style realtime event vocabulary as the real relay does (checked live with the sponsored key):
session.created/updated, server VAD (speech_started after ~1 s of mic packets, speech_stopped + committed +
input_audio_transcription.completed 0.4 s later), scripted turns (a set_intent tool call or an audio reply, optionally "heard": the transcript sent for that turn),
function_call_output + response.create -> audio reply, response.done with usage (null when cancelled by barge-in), and
the relay's rule that an image is refused until audio has been appended. Not a model: every turn comes from `script`.
"""
import argparse, asyncio, base64, json, math, sys, threading, time, uuid

TONE = base64.b64encode(bytes(
    int(3000 * math.sin(2 * math.pi * 440 * i / 24000)).to_bytes(2, "little", signed=True)[k]
    for i in range(2400) for k in range(2))).decode()          # 100 ms of 440 Hz at 24 kHz int16


class Mock:
    def __init__(self, script=None, vad_packets=25):
        self.script = list(script or [])
        self.vad_packets = vad_packets
        self.sessions = 0
        self.stats = {"audio_appends": 0, "images": 0, "images_refused": 0, "tool_outputs": [], "messages": [],
                      "response_creates": 0, "sessions": 0, "session_updates": []}
        self.kick = None            # asyncio.Event: tests can force the next scripted turn

    async def handler(self, ws):
        self.sessions += 1; self.stats["sessions"] = self.sessions
        await ws.send(json.dumps({"type": "session.created", "session": {"id": f"sess_{uuid.uuid4().hex[:8]}"}}))
        appends_since = 0; talking = None; speaking = False
        self.kick = asyncio.Event()

        async def audio_reply(text="Okay.", chunks=5, chunk_ms=100):
            nonlocal talking
            rid = f"resp_{uuid.uuid4().hex[:8]}"
            await ws.send(json.dumps({"type": "response.created", "response": {"id": rid}}))
            try:
                for _ in range(chunks):
                    await ws.send(json.dumps({"type": "response.audio.delta", "response_id": rid, "delta": TONE}))
                    await asyncio.sleep(chunk_ms / 1000)
                await ws.send(json.dumps({"type": "response.audio_transcript.done", "response_id": rid, "transcript": text}))
                await ws.send(json.dumps({"type": "response.done", "response": {"id": rid, "status": "completed",
                                          "usage": {"input_tokens": 300, "output_tokens": 40, "total_tokens": 340}}}))
            except asyncio.CancelledError:
                await ws.send(json.dumps({"type": "response.done", "response": {"id": rid, "status": "cancelled", "usage": None,
                                          "status_details": {"type": "cancelled", "reason": "turn_detected"}}}))
                raise
            finally:
                talking = None

        async def next_turn():
            if not self.script:
                return await audio_reply("Mm-hm.", 2)
            item = self.script.pop(0)
            if "tool" in item:
                rid = f"resp_{uuid.uuid4().hex[:8]}"; cid = f"call_{uuid.uuid4().hex[:8]}"
                await ws.send(json.dumps({"type": "response.created", "response": {"id": rid}}))
                await ws.send(json.dumps({"type": "response.function_call_arguments.done", "response_id": rid,
                                          "call_id": cid, "name": "set_intent", "arguments": json.dumps(item["tool"])}))
                await ws.send(json.dumps({"type": "response.output_item.done", "response_id": rid,
                                          "item": {"type": "function_call", "call_id": cid, "name": "set_intent",
                                                   "arguments": json.dumps(item["tool"])}}))
                await ws.send(json.dumps({"type": "response.done", "response": {"id": rid, "status": "completed",
                                          "usage": {"input_tokens": 500, "output_tokens": 30, "total_tokens": 530}}}))
            else:
                await audio_reply(item.get("text", "Okay."), item.get("audio_chunks", 5), item.get("chunk_ms", 100))

        async def kicker():
            while True:
                await self.kick.wait(); self.kick.clear()
                await next_turn()
        kt = asyncio.create_task(kicker())
        try:
            async for raw in ws:
                ev = json.loads(raw); t = ev.get("type")
                if t == "session.update":
                    self.stats["session_updates"].append(ev["session"])
                    await ws.send(json.dumps({"type": "session.updated", "session": ev["session"]}))
                elif t == "input_audio_buffer.append":
                    self.stats["audio_appends"] += 1; appends_since += 1
                    self.stats["audio_since_commit"] = self.stats.get("audio_since_commit", 0) + 1
                    if not speaking and appends_since >= self.vad_packets:
                        speaking = True; appends_since = 0
                        await ws.send(json.dumps({"type": "input_audio_buffer.speech_started"}))
                        if talking: talking.cancel()                    # barge-in cancels the current reply
                    elif speaking and appends_since >= self.vad_packets * 0.4:
                        speaking = False; appends_since = 0
                        iid = f"item_{uuid.uuid4().hex[:8]}"
                        await ws.send(json.dumps({"type": "input_audio_buffer.speech_stopped", "item_id": iid, "audio_end_ms": 1400}))
                        await ws.send(json.dumps({"type": "input_audio_buffer.committed", "item_id": iid}))
                        self.stats["audio_since_commit"] = 0
                        heard = (self.script[0].get("heard") if self.script else None) or "mock transcript"
                        await ws.send(json.dumps({"type": "conversation.item.input_audio_transcription.completed", "item_id": iid,
                                                  "transcript": heard, "language": "en", "emotion": "neutral"}))
                        await next_turn()
                elif t == "input_image_buffer.append":
                    if not self.stats.get("audio_since_commit"):    # the real relay: "Error append image before append audio"
                        self.stats["images_refused"] += 1
                        await ws.send(json.dumps({"type": "error", "error": {"type": "invalid_request_error",
                                                  "message": "Error append image before append audio."}}))
                    else:
                        self.stats["images"] += 1
                elif t == "conversation.item.create":
                    item = ev.get("item", {})
                    if item.get("type") == "function_call_output": self.stats["tool_outputs"].append(item)
                    else: self.stats["messages"].append(item)
                elif t == "response.create":
                    self.stats["response_creates"] += 1
                    if self.stats["messages"] and self.stats["messages"][-1].get("_pending", True):
                        self.stats["messages"][-1]["_pending"] = False
                    talking = asyncio.create_task(audio_reply("Okay.", 3))
                elif t == "mock.close":
                    await ws.close(code=1001, reason="mock close")
                elif t == "mock.stats":
                    await ws.send(json.dumps({"type": "mock.stats", **{k: v for k, v in self.stats.items() if k != "session_updates"}}))
        except Exception as e:                       # peer killed mid-turn (tests, Ctrl+C): not a mock failure
            if "Closed" not in e.__class__.__name__ and not isinstance(e, ConnectionError): raise
        finally:
            kt.cancel()

    def serve_in_thread(self, port):
        """Run the server on a background thread; returns (thread, stop function)."""
        import websockets
        loop = asyncio.new_event_loop(); stop = loop.create_future()

        async def main():
            async with websockets.serve(self.handler, "127.0.0.1", port):
                await stop
        def run(): loop.run_until_complete(main())
        th = threading.Thread(target=run, daemon=True); th.start()
        time.sleep(0.3)
        return th, lambda: loop.call_soon_threadsafe(stop.set_result, None)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--script", default='[{"tool": {"intent": "follow_me"}}, {"text": "Sure, what else?", "audio_chunks": 8}]')
    a = ap.parse_args()
    m = Mock(json.loads(a.script))
    th, stop = m.serve_in_thread(a.port)
    print(f"[mock] ws://127.0.0.1:{a.port}/v1/realtime   script: {a.script}")
    try:
        while True: time.sleep(1)
    except KeyboardInterrupt:
        stop()
