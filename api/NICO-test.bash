source activate.bash
source pyrep_env.bash

for MODULE in nicoface nicomotion
do
  echo Testing $MODULE
  pytest src/$MODULE/tests
done
