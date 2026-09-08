# Configuration Validation file tools

These tools in here help you generate a PDF for review of the configuration.
The actual configs lie in `{flight,rfs}-computer/configs`.

The two defining variables are the *binary commit* as well as the *config commit*.

- The *binary commit* is the git commit hash in which clean worktree the binary is built.
- The *config commit* is the blob hash (from `git hash-object`) of the config.

The validation file then shows the full config content with differences to a base file highlighted.

## Create PDF

There are two steps:

1. generate the metadata.
    ```sh
    python3 ./configuration/gen_metadata.py $PATH_TO_CONFIG $PATH_TO_CONFIG_TEMPLATE $BINARY_COMMIT > ./configuration/out/data.json 
    ```

    Noteably, the git commit of the binary can be HEAD for the current version, or a commit hash.
    Or you can use PATH_TO_CONFIG_TEMPLATE = PATH_TO_CONFIG if no diff is required.

2. build the pdf.
    ```sh
    docker run -v .:/workspace --rm ghcr.io/typst/typst:latest compile --root /workspace --input metadata-file=out/data.json /workspace/configuration/validation_file_template.typ /workspace/configuration/out/validation_file.pdf
    ```

Outputs go to `configuration/out/` which is gitignored.
