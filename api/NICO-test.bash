source activate.bash
source pyrep_env.bash

if ! [ -x "$(command -v pytest)" ]; then
  echo "Pytest not found"
  pip install pytest
fi

for MODULE in nicoaudio nicoface nicomotion
do
  echo Testing $MODULE
  pytest -v src/$MODULE/tests
done
