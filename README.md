# GPU Parking Demo

## Developing

clone and cd into this repo;

```bash
$ git submodule init
$ git submodule update
$ cd parking/pkphysx; PYMODE=debug pip install -e . ; cd ..
$ python test.py
```

### PhysX Documentation

```bash
python -m http.server -d ${PHYSX_ROOT}/physx
```

### Knowledge points

#### 给定坐标点创建Polygon

Cooking Triangle Meshes

```cpp
PxTriangleMeshDesc meshDesc;
meshDesc.points.count           = nbVerts;
meshDesc.points.stride          = sizeof(PxVec3);
meshDesc.points.data            = verts;

meshDesc.triangles.count        = triCount;
meshDesc.triangles.stride       = 3*sizeof(PxU32);
meshDesc.triangles.data         = indices32;

PxDefaultMemoryOutputStream writeBuffer;
PxTriangleMeshCookingResult::Enum result;
bool status = cooking.cookTriangleMesh(meshDesc, writeBuffer,result);
if(!status)
    return NULL;

PxDefaultMemoryInputData readBuffer(writeBuffer.getData(),
    writeBuffer.getSize());
return physics.createTriangleMesh(readBuffer);
```
#### 创建triangle mesh时少了三个三角？
?

#### 注意在创建TriangleMesh时区分是否直接加入PxPhysics
```cpp
PxTriangleMesh* triMesh = NULL;
PxU32 meshSize = 0;

// The cooked mesh may either be saved to a stream for later loading, or inserted directly into PxPhysics.
if (inserted)
{
  triMesh = gCooking->createTriangleMesh(meshDesc, gPhysics->getPhysicsInsertionCallback());
}
else
{
  PxDefaultMemoryOutputStream outBuffer;
  gCooking->cookTriangleMesh(meshDesc, outBuffer);

  PxDefaultMemoryInputData stream(outBuffer.getData(), outBuffer.getSize());
  triMesh = gPhysics->createTriangleMesh(stream);

  meshSize = outBuffer.getSize();
}
```
#### 创建车道

- 用d6Joints将物体固定在平面上
   - `include/Scene.h``class Scene`固定了重力
- 物体的filter mask bits
- 弹性等参数配置
- userData的设计（以及取出时的规则）

#### 创建、删除车位

- 将车位绑定在车道上
- 随时删除joint

#### 车道长度重新设置

```cpp

```

#### joint超出阈值后断开

#### clearForces、ApplyForce，或直接设置forcefield

#### Ray Casting

#### 范围查询

#### Box重设大小

- 将lanes用Deformable meshes重新实现

#### 创建Shape时的flags

```cpp
// PxShape.h
/**
\brief Flags which affect the behavior of PxShapes.

@see PxShape PxShape.setFlag()
*/
struct PxShapeFlag
{
    enum Enum
    {
        /**
        \brief The shape will partake in collision in the physical simulation.

        \note It is illegal to raise the eSIMULATION_SHAPE and eTRIGGER_SHAPE flags.
        In the event that one of these flags is already raised the sdk will reject any 
        attempt to raise the other.  To raise the eSIMULATION_SHAPE first ensure that 
        eTRIGGER_SHAPE is already lowered.

        \note This flag has no effect if simulation is disabled for the corresponding actor (see #PxActorFlag::eDISABLE_SIMULATION).

        @see PxSimulationEventCallback.onContact() PxScene.setSimulationEventCallback() PxShape.setFlag(), PxShape.setFlags()
        */
        eSIMULATION_SHAPE                = (1<<0),

        /**
        \brief The shape will partake in scene queries (ray casts, overlap tests, sweeps, ...).
        */
        eSCENE_QUERY_SHAPE                = (1<<1),

        /**
        \brief The shape is a trigger which can send reports whenever other shapes enter/leave its volume.

        \note Triangle meshes and heightfields can not be triggers. Shape creation will fail in these cases.

        \note Shapes marked as triggers do not collide with other objects. If an object should act both
        as a trigger shape and a collision shape then create a rigid body with two shapes, one being a 
        trigger shape and the other a collision shape.     It is illegal to raise the eTRIGGER_SHAPE and 
        eSIMULATION_SHAPE flags on a single PxShape instance.  In the event that one of these flags is already 
        raised the sdk will reject any attempt to raise the other.  To raise the eTRIGGER_SHAPE flag first 
        ensure that eSIMULATION_SHAPE flag is already lowered.

        \note Trigger shapes will no longer send notification events for interactions with other trigger shapes.

        \note Shapes marked as triggers are allowed to participate in scene queries, provided the eSCENE_QUERY_SHAPE flag is set. 

        \note This flag has no effect if simulation is disabled for the corresponding actor (see #PxActorFlag::eDISABLE_SIMULATION).

        @see PxSimulationEventCallback.onTrigger() PxScene.setSimulationEventCallback() PxShape.setFlag(), PxShape.setFlags()
        */
        eTRIGGER_SHAPE                    = (1<<2),

        /**
        \brief Enable debug renderer for this shape

        @see PxScene.getRenderBuffer() PxRenderBuffer PxVisualizationParameter
        */
        eVISUALIZATION                    = (1<<3)
    };
};
```

#### Shape由两个Actor共用？

`PxShape::isExclusive`为false？



### 2D可视化部分

#### world.Step是否有额外配置

#### DrawDebugData

- to_screen实现

#### mouseJoint? 或许可以通过Controller实现

#### 统计信息：bodyCount, contactCount, jointCount, proxyCount

#### mousedown: AABB选择

#### python: Vec2

### pybind11高级

## Install
