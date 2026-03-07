# Configuration Validation file tools

These tools in here help you generate a PDF for review of the configuration.

The baseline is the *binary commit* as well as the *config commit*.

- The *binary commit* is the git commit hash in which clean worktree the binary is built.
- The *config commit* is the blob hash (from `git hash-object`).

## Create PDF

There are two steps:

1. generate the metadata.
    ```sh
    python3 ./configuration/gen_metadata.py $PATH_TO_CONFIG> $PATH_TO_CONFIG_TEMPLATE $BINARY_COMMIT > ./configuration/data.json 
    ```

    noteably, the git commit of the binary can be HEAD for the current version, or a commit hash.

2. build the pdf.
    ```sh
    docker run -v .:/workspace --rm ghcr.io/typst/typst:latest compile --input metadata-file=data.json /workspace/configuration/validation_file_template.typ /workspace/configuration/validation_file.pdf
    ```

Don't commit the changes, delete them afterwards
