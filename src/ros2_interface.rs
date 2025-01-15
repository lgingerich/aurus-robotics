use r2r::*; // r2r is a Rust client library for ROS 2

pub struct RobotROS2Interface {
    node: Node,
    joint_state_pub: Publisher<JointState>,
    pose_pub: Publisher<PoseStamped>,
    command_sub: Subscription<String>,
}

impl RobotROS2Interface {
    pub fn new() -> Result<Self, &'static str> {
        let ctx = Context::create()?;
        let node = Node::create(ctx, "robot_controller", "")?;

        // Publishers
        let joint_state_pub = node.create_publisher::<JointState>("/joint_states", QosProfile::default())?;
        let pose_pub = node.create_publisher::<PoseStamped>("/robot_pose", QosProfile::default())?;
        
        // Subscribers
        let command_sub = node.subscribe::<String>("/robot_commands", QosProfile::default())?;

        Ok(Self {
            node,
            joint_state_pub,
            pose_pub,
            command_sub,
        })
    }

    pub fn publish_state(&self, robot: &Robot) -> Result<(), &'static str> {
        // Publish joint states
        let msg = JointState {
            header: Header::default(),
            name: vec!["joint1".into(), "joint2".into(), "joint3".into()],
            position: vec![0.0, 0.0, 0.0], // Get from robot state
            velocity: vec![0.0, 0.0, 0.0],
            effort: vec![0.0, 0.0, 0.0],
        };
        self.joint_state_pub.publish(&msg)?;

        Ok(())
    }
} 