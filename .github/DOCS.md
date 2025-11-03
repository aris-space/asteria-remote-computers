# CI workflows

The CI tests the rust code for compilation, runs tests and checks lints.

Sources: 
- <https://www.youtube.com/watch?v=xUH-4y92jPg&t=491s&ab_channel=JonGjengset> (05.10.2024) 
- <https://github.com/jonhoo/rust-ci-conf/tree/main> (05.10.2024)
- <https://github.com/actions/checkout/issues/116#issuecomment-644419389> (13.2.2025)

# Setup

The repository and all its submodules must be added to the PAT token `ASTERIA_CI_TOKEN`, contact @Indeximal.

# Locally equivalent commands

Assure that the following 4 commands generate no warnings.

```sh
cargo test --locked --workspace --all-features -- --skip skipci
cargo fmt --check
cargo clippy --workspace --all-targets --all-features -- -Dwarnings
cargo doc --no-deps --all-features --workspace --lib --examples --bins --document-private-items
```
