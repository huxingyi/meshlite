![appveyor status](https://ci.appveyor.com/api/projects/status/github/huxingyi/meshlite?branch=master&svg=true)

## meshlite
WIP

## Check generated `.obj` in blender
```
Mac:
RUST_BACKTRACE=1 cargo run && /Applications/blender.app/Contents/MacOS/blender --python show_obj_in_blender.py -- test.obj
```
