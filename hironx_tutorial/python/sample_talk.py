#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from sound_play.msg import SoundRequestAction, SoundRequestGoal

class speak_class:
    
    def __init__(self):
        self.volume = 1.0
        self.command = 1
        self.sound = -3
        self.arg2 = 'ja'
        self.str_candidate = ''
        rospy.init_node('hiro_talk')
        self.sub = rospy.Subscriber('/speech_to_text', SpeechRecognitionCandidates, self.talk_cb)
        self.actionlib_part = actionlib.SimpleActionClient('/robotsound_jp', SoundRequestAction)
        self.actionlib_part.wait_for_server()
        #rospy.Timer(rospy.Duration(10), self.loopOnce)

    def talk_cb(self, msg):
        #print("talk_cb")
        if msg.transcript:
            for char in msg.transcript:
                self.str_candidate += char
            print("got words {}".format(self.str_candidate))
            
            print("PC speaks")
            speak_msg = SoundRequestGoal()
            speak_msg.sound_request.volume = self.volume
            speak_msg.sound_request.command = self.command
            speak_msg.sound_request.sound = self.sound

            if "はい" in self.str_candidate or "そうです" in self.str_candidate:
                speak_msg.sound_request.arg = "よかったです"
            elif "いいえ" in self.str_candidate or "違います" in self.str_candidate:
                speak_msg.sound_request.arg = "ごめんなさい、次は頑張ります"
            elif "さようなら" in self.str_candidate:
                speak_msg.sound_request.arg = "さようなら"
                self.sub.unregister()
            else:
                speak_msg.sound_request.arg = "さっき、しゃべったのは、" + self.str_candidate + "で合っていますか？"
            print(speak_msg.sound_request.arg)

            speak_msg.sound_request.arg2 = self.arg2
            self.actionlib_part.send_goal(speak_msg)
            self.str_candidate = ""
        

if __name__ == '__main__':
    try:
        hiro_talk = speak_class()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


       




         


