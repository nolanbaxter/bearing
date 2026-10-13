# Security

## Supported versions

The latest release of Bearing is supported: fixes land in the next patch of the current minor
version. Older versions are not patched. Until the first release, `develop` is what is supported.

## Reporting a vulnerability

Please report it privately, not in a public issue: on the repository's **Security** tab, choose
**Report a vulnerability**
([direct link](https://github.com/nolanbaxter/bearing/security/advisories/new)). Say what is
affected, how to reproduce it, and what an attacker could do with it.

You will hear back within a week. A confirmed issue is fixed in a patch release and credited to
you in its changelog, unless you would rather it were not.

## What counts

Bearing runs in the page or process that imports it and reads no files, makes no requests and
starts nothing that keeps running, so most of what it does is bounded by its host. Reports that
matter most:

- input that makes Bearing run code, or read or write memory it shouldn't. From milestone 9 it
  reads `.glb` files, and from milestone 15 saved worlds: a crafted one of either belongs here;
- anything in the release workflow or the published package that could let someone other than
  the maintainer publish a version.

Input that only throws, a scene that only runs slowly, or a simulation that only goes wrong is a
bug: open an issue for it.
