# follow_line_challenge

+108 mm 

bot changes:
 added 
         <joint name="${name}_pen_pose" type="fixed">
            <origin xyz="0.000 0 0.0108" rpy="0.0 0.0 0.0" />
            <parent link="${name}_tip_joint" />
            <child link="${name}_pen"/>
        </joint>
        <link name="${name}_pen">
            <inertial>
                <mass value="0.001" />
                <origin xyz="0 0 0" />
                <inertia ixx="0.000001" ixy="0.0" ixz="0.0" iyy="0.000001" iyz="0.0" izz="0.000001" />
            </inertial>
        </link>
 to gripper.udrf.xacro

 xyz="-0.02 0 -0.04" rpy="0 -1.60 0"
