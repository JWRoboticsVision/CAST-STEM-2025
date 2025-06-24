from pathlib import Path
import shutil
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[4]

def add_path(path):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))


def make_clean_folder(folder_path):
    folder = Path(folder_path)
    if folder.exists():
        shutil.rmtree(folder)
    folder.mkdir(parents=True)
