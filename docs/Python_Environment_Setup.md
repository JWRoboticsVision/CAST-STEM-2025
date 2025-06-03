# Python Environment Setup

## Install Conda

Install miniconda following instruction from the [official website](https://docs.conda.io/en/latest/miniconda.html).

## Create a new environment of Python 3.10

```bash
conda create -n summer_camp python=3.10
```

## Activate the environment

```bash
conda activate summer_camp
```

## Install required packages

- Install packages from the `requirements.txt` file

  This file contains basic packages that are required for the project.

  ```bash
  python -m pip install -r requirements.txt --no-cache-dir
  ```

## Deactivate the environment

```bash
conda deactivate
```

## Useful commands for managing the environment

- Remove the environment

```bash
conda remove -n summer_camp --all
```

- Update the environment

```bash
conda update -n summer_camp --all
```

- List all environments

```bash
conda env list
```

- List all packages in the environment

```bash
conda list
```

- Export the environment

```bash
conda env export > environment.yml
```

- Create an environment from the file

```bash
conda env create -f environment.yml
```
