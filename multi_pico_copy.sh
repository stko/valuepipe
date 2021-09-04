for ((i = 2; i <= $#; i++ )); do
  cp -v $1 ${!i}
done