import os
import subprocess

BASE_URL = "https://github.com/thkrrc1"

def run(cmd):
    print(f"$ {cmd}")
    result = subprocess.run(cmd, shell=True)
    if result.returncode != 0:
        raise RuntimeError(f"コマンド失敗: {cmd}")

def main():
    project_name = input("クローンしたいロボット名を入力してください: ").strip()
    repo_url = f"{BASE_URL}/{project_name}.git"
    clone_dir = project_name

    if os.path.exists(clone_dir):
        print(f"{clone_dir} が既に存在します。削除して再cloneします。")
        subprocess.run(f"rm -rf {clone_dir}", shell=True)

    run(f"git clone {repo_url}")

if __name__ == "__main__":
    main()

