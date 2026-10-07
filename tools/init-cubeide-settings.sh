#!/bin/sh
# Seed local CDT provider settings without replacing an existing local file.
set -eu

repo_root=$(git rev-parse --show-toplevel)
cd "$repo_root"

settings_file=.settings/language.settings.xml
if [ -e "$settings_file" ] || [ -L "$settings_file" ]; then
    printf '%s\n' 'Existing local language.settings.xml retained.'
else
    cp .settings/language.settings.xml.template "$settings_file"
    printf '%s\n' 'Local language.settings.xml created from the shared template.'
fi
