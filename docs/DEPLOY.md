# Environment & deployment

The APM software has been developed on machines with **different CPU architectures**: the `x86_64` dev PC and the `aarch64` ZED Box Orin NX 16GB. Because of that, a build-pinned `conda env export` is **not**
portable between them. Instead, [`environment.yml`](../environment.yml) is the curated source
of truth (direct deps only), and `pyzed` is installed separately from the ZED SDK.

## Create / update the environment

```bash
# First time:
conda env create -f environment.yml          # creates the `apm` env

# After editing environment.yml:
conda env update -f environment.yml --prune   # adds/removes to match the file
```

Run everything in that env (e.g. `conda run -n apm python -m apm`), or `conda activate apm`.

> **Rule:** add new dependencies to `environment.yml`, then `env update` - don't `pip install`
> /`conda install` ad hoc. Install conda-forge packages via conda; use the `pip:` section only
> for packages not on conda-forge (currently just `gpsdclient`). Mixing pip over conda for the
> same package (e.g. a pip `numpy` on top of conda's) causes ABI mismatches.

## pyzed (APM only)

`pyzed` is **not** on PyPI or conda - it ships with the ZED SDK and must match the installed
SDK + CUDA/JetPack + the env's Python version. The current APM runs **ZED SDK 5.1 → `pyzed` 5.1
on Python 3.13** (aarch64/Orin). Install it *into the env* after creating it:

```bash
conda activate apm
python /usr/local/zed/get_python_api.py        # builds & installs the pyzed wheel for this Python
python -c "import pyzed.sl as sl; print(sl.Camera)"   # verify
```

If `get_python_api.py` reports an unsupported Python, pin a SDK-supported version in a
separate APM copy of `environment.yml` (e.g. `python=3.11`) and recreate the env.

> **Order matters! Avoid the numpy ABI footgun.** `get_python_api.py` will `pip install` numpy
> (and `cython`) while building the wheel, which shadows conda's numpy if it isn't already
> present. Always create the conda env **first** (so conda owns numpy), **then** run
> `get_python_api.py`. Afterwards, verify there is exactly one numpy - conda's:
>
> ```bash
> conda list numpy        # should show the conda build
> pip list | grep numpy   # should be EMPTY - a pip numpy here is shadowing conda's
> ```
>
> If pip shadowed it, `conda install --force-reinstall numpy` to restore the conda build.
> (`cython` is only needed to build the wheel and is otherwise harmless to leave.)

## Cleaning up the current dev env (one-time)

The existing `base` env has packages installed by **both** pip and conda (e.g. pip `numpy`
shadowing conda's `numpy-base`). Cleanest fix is to stop using `base` and build a fresh `apm`
env from the spec:

```bash
conda env create -f environment.yml
conda run -n apm python -m pytest -q          # sanity check
```

## Reproducible, per-arch locks (optional, recommended for real deploys)

`environment.yml` uses version floors, so the two machines may resolve slightly different
builds. For exact reproducibility, render a lock per platform with
[conda-lock](https://github.com/conda/conda-lock) (it can solve for an arch you're not on):

```bash
conda install -n base -c conda-forge conda-lock
conda-lock lock -f environment.yml -p linux-64 -p linux-aarch64   # writes conda-lock.yml

# Deploy from the lock (exact, fast, no solving):
conda-lock install -n apm conda-lock.yml      # picks the current platform automatically
```

Commit `conda-lock.yml` alongside `environment.yml`. Regenerate it whenever `environment.yml`
changes. `pyzed` still installs separately (it isn't in any index).
