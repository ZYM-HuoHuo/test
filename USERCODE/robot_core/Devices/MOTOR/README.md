### How to use
推荐使用'git subtree'的方式进行移植
```bash
cd 根目录
git stash

git subtree add --prefix=robot_core/Devices/MOTOR git@e.coding.net:scnu-pioneer/ec/MOTOR.git 2025_drv_motor --squash
```

### 更新
```bash
git stash

git subtree pull --prefix=robot_core/Devices/MOTOR git@e.coding.net:scnu-pioneer/ec/MOTOR.git 2025_drv_motor --squash

git stash pop
```
