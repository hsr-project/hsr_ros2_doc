******************
音声合成を行う
******************

HSRでは音声合成によりロボットに発話させることができます。

シミュレータでは音声を発することができないので、代わりに文字で確認します。

1. 以下を実行し、確認用のターミナルを起動してください。
   
   .. code-block:: bash

      $ ros2 topic echo /talk_request
   
#. 以下を実行し、発話させるテキストをコンソールから送信してください。
   
   .. ifconfig:: language=='ja'
   
      .. code-block:: bash
   
         In []: tts.say(u'こんにちは。HSRだよ。')
   
   .. ifconfig:: language!='ja'
   
      .. code-block:: bash
   
         In []: tts.language = tts.ENGLISH
         In []: tts.say(u'Hi. My name is HSR.')
   
#. 確認用のターミナルに、以下が表示されます。
   
   .. ifconfig:: language=='ja'
   
      .. code-block:: bash
   
            $ ros2 topic echo /talk_request
            interrupting: false
            queueing: false
            language: 0
            sentence: こんにちは。HSRだよ。
   
   .. ifconfig:: language!='ja'
   
      .. code-block:: bash
   
            $ ros2 topic echo /talk_request
            interrupting: false
            queueing: false
            language: 1
            sentence: Hi. My name is HSR.
