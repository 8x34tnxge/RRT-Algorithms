export PYTHONPATH=".:src/"

pytest test\
       	--color=yes --code-highlight=yes\
	--log-file=test.log --log-level=DEBUG
