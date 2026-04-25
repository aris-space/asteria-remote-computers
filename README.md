# ASTERIA Remote Computers

This repository contains the code for the flight computer of the rocket and the RFS computer in the remote filling station for the **ASTERIA** sounding rocket project by
[ARIS Space](https://aris-space.ch/).
They share code using the crates in `lib-remote-computers`.

# Legacy

This code was copied and merged from
- <https://github.com/aris-space/hermes-flight-computer>
- <https://github.com/aris-space/hermes-rfs-computer>
- <https://github.com/aris-space/hermes-lib-remote-computers>

Because this meant commits now contained broken PR links and merging the history would have required major git surgery,
I have opted to instead squash everything into initial commits with the proper Co-authorships.

## Procedures

The updated procedures are on this git repository:
<https://github.com/aris-space/asteria-procedures-backup/>

## Building
// TODO: do we want to include  building instructions since we already have them in the procedures? At least with which flags to do that imo.

## Squash procedure

In general, a hard squash like this is essentially equivalent to:

1. Getting all authors: `git log --format="%aN <%aE>" --no-merges | sort | uniq`
2. `rm -rf .git && git init`
3. create an initial commit where you then just append `Co-authored-by: NAME <EMAIL>` to the commit message for every author given above

## Licence 
TODO: GPLv3? or MIT/Apache2.0?