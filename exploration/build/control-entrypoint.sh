#!/usr/bin/env bash
set -eo pipefail
exec /usr/local/bin/p9explore-entrypoint bash /opt/p9-exploration/control/p9explore-control "$@"
