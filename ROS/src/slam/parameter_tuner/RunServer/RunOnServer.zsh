#bin/zsh

pip install -r requirement.txt
roslaunch parameter_tuner tuner.launch
curl -X POST -H 'Content-type: application/json' --data '{"text":"Parameter is finished"}' https://hooks.slack.com/services/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
