# Viewer Development Tips

这部分可以直接参考[pybox2d中的GUI实现](https://github.com/pybox2d/pybox2d/blob/master/library/Box2D/examples/framework.py)：

## 相关目录
- `pybox2d/library/Box2D/examples/`，直接对应本项目的`/viewer/`:
  - `/framework.py`:
    实现`FrameworkBase`，这部分与物理引擎耦合，可能要参考`pkphysx`中的`pyphysx_render/pyrender_base.py`里的`PyRenderBase.update`中处理`PhysX actor`的方式。`FrameworkBase`中的`self.world`基本可以替换为`pkphysx`中的`Scene`类型。
  - `/settings.py`: GUI的一些设置，可以直接引入
  - `/pgu/`: pygame的窗口设置，可以直接引入
  - `/backends/`: 正常而言应该只涉及具体的gui框架，但在`pybox2d`的实现中其中的`_framework`代码对`pybox2d`库有依赖，须手动替换为`pkphysx`中的对应元素。暂时不清楚是否存在更深的耦合。
  - 杂项: 均为继承Framework的示范，可以用来参考`test_viewer.py`的写法

## 已知的对应关系

- `b2world`: `pkphysx.Scene`
  - `?`: `?`


## 测试方法
在`viewer/test_viewer.py`的`test_viewer(ns)`中编写测试代码，
然后
```bash
python test.py (-t|--test) viewer [-f <geojson filename>]
```
