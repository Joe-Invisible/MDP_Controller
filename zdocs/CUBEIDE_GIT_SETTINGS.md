# CubeIDE configuration and Git

## Local language settings

`.settings/language.settings.xml` is no longer tracked. Its `env-hash`
fingerprints the local compiler environment and changes across machines and
toolchain installations. The file also contains language-provider definitions,
so a tracked `.settings/language.settings.xml.template` preserves the project's
existing provider defaults without the environment fingerprints.

After adopting this change or cloning the repository, run the following before
opening the project in CubeIDE:

```sh
sh tools/init-cubeide-settings.sh
```

Use Git Bash on Windows. The script works from any directory inside the
repository and creates the local settings file only if it is absent.
It preserves an existing file and does not modify Git configuration or staging.
CubeIDE can then update the ignored local file without working-tree noise.

When merging the removal of a tracked file, Git may remove the old local copy.
If it contains intentional local provider customizations, save a copy outside
the repository before merging, then restore it afterwards instead of using
the template. The repository's previous tracked file contained standard
provider definitions and two environment hashes; those definitions are retained
in the template.

If shared provider settings or build configuration IDs change, update the
template deliberately. Existing local files are never automatically overwritten.

## Shared build settings

Keep `.cproject`, `.project`, and the `.ioc` file tracked. The `.cproject`
file contains include paths, macros, compiler and linker options, source
exclusions, and per-file settings. Repository commits labelled as configuration
noise have included source exclusions and per-file compiler changes.

Review residual changes with:

```sh
git diff -- .cproject
```

Discard only hunks confirmed to be incidental. Do not hide the whole file using
`assume-unchanged` or `skip-worktree`, and do not ignore all of `.settings/`.
The latter may contain other shared project settings.

A clean filter that strips hashes was tested and rejected for this task:
although it suppresses the textual diff, Git can still report a modified file
when the working-copy size changes. Ignoring the live language-settings file
eliminates that working-tree noise directly.
