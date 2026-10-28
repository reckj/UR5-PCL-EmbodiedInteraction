# students/ — your work goes here

Create your own folder and keep all your scripts in it:

    students/<your-name>/

For example `students/anna/first_moves.py`. Start by copying an example:

```bash
mkdir -p students/anna
cp examples/grid_process.py students/anna/first_moves.py
```

Run it from the dev shell (`docker compose run --rm dev bash`):

```bash
python students/anna/first_moves.py
```

The whole repo is mounted into the container, so your folder is visible there
without any copying. Everything under `students/` is ignored by git on purpose
— `git pull` never touches your files, and your work is not pushed to the
course repo. Back it up yourself (your own git repo, a USB stick, …).

Don't edit files in `src/`, `scripts/` or `examples/` — they are replaced on the
next `git pull`. Copy, then change the copy.
