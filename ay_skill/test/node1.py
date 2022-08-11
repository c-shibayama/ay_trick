#!/usr/bin/python
from core_tool import *
def Help():
  return '''Test of using a SCRIPT as a standalone ROS node.
  Usage: We assume two usages:
  Executing on CUITool:
    > test.node1 TOPIC_NAME, LAMBDA
    Result: values are published through: /cui_tool****/TOPIC_NAME.
  Executing as a standalone ROS node:
    $ rosrun ay_trick direct_run.py 'test.node1 TOPIC_NAME, LAMBDA' __name:=NODE_NAME
    Result: values are published through: /NODE_NAME/TOPIC_NAME.
  The arguments are common:
    TOPIC_NAME: Name of a topic.
    LAMBDA: Equation of x.
    e.g. "test_topic", lambda x:np.cos(x)
  Note:
    When executing on CUITool, you can quit this script with Ctrl+C,
    but it also shutdowns the CUITool itself.
'''

def Run(ct,*args):
  topic_name= args[0]
  eq= args[1]
  ct.AddPub(topic_name, '~{}'.format(topic_name), std_msgs.msg.Float64)
  t_start= rospy.Time.now()
  while not rospy.is_shutdown():
    t_elapsed= (rospy.Time.now()-t_start).to_sec()
    value= eq(t_elapsed)
    ct.pub[topic_name].publish(value)
    print 'at {}, value: {}'.format(t_elapsed,value)
    rospy.sleep(0.1)
