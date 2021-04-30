Changes from iCubV3 model
=====================

The iCub3 model available in the `icub-models` repository has all the collisions enabled. In some particular situations, e.g.: shoulder joints, this leads to contacts happening between its own parts.

Changing this model required a bit more tinkering due to two factors: 
- we had to deactivate the collisions between consecutive parts in the arms;
- we had to introduce a contact sensor in the URDF file (not as simple as in the SDF).

In order to remove the collisions, we had to change the links for the shoulder parts (`l_shoulder_1`, `l_shoulder_2`, `l_shoulder_3`, `r_shoulder_1`, `r_shoulder_2`, `r_shoulder_3`) and the elbow (`l_elbow` and `r_elbow`) and forearm (`l_forearm` and `r_forearm`) links. We removed the collisions sections of these links, does preventing gazebo from computing collisions with these surfaces.

In order to detect the contacts, we need to tell Gazebo to consider a contact sensor with the geometry of the part. To do this, we need to create a `<gazebo>` section in the end:

```
<gazebo reference="l_upper_arm">
  <selfCollide>true</selfCollide>
  <collision>
    <origin xyz="0.030743149468679197 -0.14153106591133113 0.06954758245258447" rpy="-0.0715644949881 -0.0619699884186 -1.29535744982"/>
    <geometry>
      <mesh>
        <scale>0.001 0.001 0.001</scale>
        <uri>model://iCub/meshes/simmechanics/sim_icub3_l_upperarm_prt.stl</uri>
      </mesh>
    </geometry>
    <surface>
      <contact>
        <collide_without_contact>true</collide_without_contact>
        <ode/>
      </contact>
      <friction>
        <ode/>
      </friction>
    </surface>
  </collision>
  <sensor name="l_upper_arm_collision_sensor" type="contact">
    <contact>
      <collision>collision_left</collision>
    </contact>
  </sensor>
</gazebo>
```

We first need to specify which link this section refers to (`<gazebo reference="l_upper_arm">`). We then need to activate the self collision, forcing Gazebo to compute collisions with the model itself (not just external models), done through `<selfCollide>true</selfCollide>`.


We then specify the geometry ("mesh") of our collision surface, along with other options like the physics calculator (ODE) and other physics properties (`collide_without_contact`, `friction`, etc).

Finally, we create our sensor, specifying the type (`contact`) and the options that come with it.


The `<gazebo>` section emulates the configuration of SDF files, thus allowing properties like the ones described above to be included in URDF files. 


With these changes our left upper arm will be able to detect contacts with other parts, **as long as those parts also have a collision tag**. 
