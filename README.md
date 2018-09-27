[![appveyor status](https://ci.appveyor.com/api/projects/status/github/huxingyi/meshlite?branch=master&svg=true)](https://ci.appveyor.com/project/huxingyi/meshlite)

## meshlite
WIP

## Check generated `.obj` in blender
```
Mac:
RUST_BACKTRACE=1 cargo run --example obj_export && /Applications/Blender/blender.app/Contents/MacOS/blender --python show_obj_in_blender.py -- test.obj
```
