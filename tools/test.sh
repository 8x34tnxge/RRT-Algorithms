export PYTHONPATH=".:src/"

for testfile in test/*.py;
do
	pytest $testfile
done
