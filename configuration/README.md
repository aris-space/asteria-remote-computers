# Configuration Validation file tools

These tools in here help you generate a PDF for review of the configuration.
The actual configs lie in `{flight,rfs}-computer/configs`.

The two defining variables are the _binary commit_ as well as the _config commit_.

- The _binary commit_ is the git commit hash in which clean worktree the binary is built.
- The _config commit_ is the blob hash (from `git hash-object`) of the config.

The validation file then shows the full config content with differences to a base file highlighted.

## Create PDF

There are two steps:

1. generate the metadata.

   ```sh
   python3 ./configuration/gen_metadata.py $PATH_TO_CONFIG $PATH_TO_CONFIG_TEMPLATE $BINARY_COMMIT > ./configuration/data.json
   ```

   Noteably, the git commit of the binary can be HEAD for the current version, or a commit hash.
   Or you can use PATH_TO_CONFIG_TEMPLATE = PATH_TO_CONFIG if no diff is required.

2. build the pdf.

   ```bash
   docker run -v .:/workspace --rm ghcr.io/typst/typst:latest compile --input metadata-file=data.json /workspace/configuration/validation_file_template.typ /workspace/configuration/validation_file.pdf

   # OR if you have Typst installed

   typst compile --input metadata-file=data.json ./configuration/validation_file_template.typ ./configuration/validation_file.pdf
   ```

Don't commit the changes, delete them afterwards
