# Organisers' usage-accounting tools (Huawei OMNI Live Challenge)

`yibu_audit.py` and `summarize_usage.py` are copied verbatim from the sponsor's example package
(`yibuapi_examples_20260918_v01.tar.gz`, linked from the key e-mail). They are kept unmodified so the report
we send back is exactly what their tool produces.

- `laptop/voice/usage_log.py` writes every Blimpy call (realtime responses and focus-watch HTTP calls) through
  `yibu_audit.append_audit_record`, into `data/omni_usage.jsonl` (schema `yibu_call_audit_v1`, key suffix only).
- `python tools/omni_report.py` runs `summarize_usage.py` over that ledger and writes the two files the organisers
  want attached to the reply e-mail (`usage_summary.json`, `usage_by_model_key_purpose.csv`) plus the e-mail text.
