# opensot_model_interface

This package replaces the good ole' xbot2_interface (https://github.com/ADVRHumanoids/xbot2_interface) which in turn
replaced the good ole' [XBotInterface](https://github.com/advrhumanoids/xbotinterface)
package by HHCM.

We try to keep the API as close as possible to the old ones.

## Quick guide

```python
import opensot_model_interface

urdf="""
<robot name="minimal">
  <link name="base_link"/>
  <link name="moving_link"/>
  <joint name="hinge_joint" type="continuous">
    <parent link="base_link"/>
    <child link="moving_link"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
"""

model = opensot_model_interface.pyxbot2_interface.ModelInterface2(urdf)
print(model.getJointNames())
print(model.q)

```
