#!/usr/bin/env python

import rospy

class VisionServer(object):
    def _handle_predict(self, request):
        print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
        return AddTwoIntsResponse(req.a + req.b)

    def add_two_ints_server(self):
        rospy.init_node('vision_server')
        s = rospy.Service('add_two_ints', AddTwoInts, self._handle_predict)
        print "Ready to add two ints."
        rospy.spin()

if __name__ == "__main__":
    VisionServer().run()