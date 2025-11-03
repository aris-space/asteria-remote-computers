# ASTERIA Remote Computers

This repository contains the code for the flight computer in the rocket and the RFS computer in the remote filling station.
They share code using the crates in `lib-remote-computers`.

# Legacy

This code was copied and merged from
- <https://github.com/aris-space/hermes-flight-computer>
- <https://github.com/aris-space/hermes-rfs-computer>
- <https://github.com/aris-space/hermes-lib-remote-computers>

Because this meant commits now contained broken PR links and merging the history would have required major git surgery,
I have opted to instead squash everything into intitial commits with the proper Co-authorships.

## Squash steps

In general, a hard squash like this is essentially equivalent to:

1. Getting all authors: `git log --format="%aN <%aE>" --no-merges | sort | uniq`
2. `rm -rf .git && git init`
3. create an initial commit where you then just append `Co-authored-by: NAME <EMAIL>` to the commit message for every author given above

I had to do some other stuff to intialize the submodules again and also change the folder structure, but otherwise this was the only git magic.
