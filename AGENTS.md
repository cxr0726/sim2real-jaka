# Project Agent Notes

## Python Environments

- Inference / policy / sim work on both PC and onboard Orin uses conda env named `teleop`.
- Teleop work on both PC and onboard Orin uses conda env named `teleop`.
- When touching inference/runtime files, verify syntax with the root project when available:
  `conda run -n teleop python -m py_compile <files>`
- When touching teleop files, verify syntax with the teleop project when available:
  `conda run -n teleop python -m py_compile <files>`

## Documentation Sync

- Keep English and Chinese documentation in sync.
- When changing a doc page, update the corresponding English and Chinese sources in the same change whenever both versions exist.
- For Docusaurus content, treat `docs/` and `docs/i18n/zh-Hans/docusaurus-plugin-content-docs/current/` as paired sources when both files exist.
- Apply the same sync rule to top-level docs such as `README.md` and `README_zh.md`.
