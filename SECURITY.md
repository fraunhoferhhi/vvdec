# Security Policy

This project does not run a coordinated/embargoed disclosure program.

## Reporting a vulnerability

File a public GitHub issue, same as any other bug. Include a minimal reproducing
bitstream if possible.

If you'd rather not disclose details publicly before a fix exists (e.g. a write
primitive or RCE-adjacent bug), you can use [private vulnerability
reporting](../../security/advisories/new) instead. We still aim to fix and
disclose promptly, just without a formal embargo period.

## Supported versions

Only the latest tagged release and `master` are supported. Older releases do not
receive backported fixes.

## Scope

VVdeC decodes untrusted bitstreams by design, so out-of-bounds reads/writes,
use-after-free, and DoS (infinite loops, excessive memory growth) triggered by
malformed input are in scope. All issues are welcome - please check for existing
duplicates first, as we get a fair number.
