export PYTHONPATH=".:src/"

pipenv run pytest test\
       	--color=yes --code-highlight=yes\
	--log-file=test.log --log-level=DEBUG\
    --timeout=600