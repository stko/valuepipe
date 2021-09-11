names=( $1 )
codeflag=0
for file in "${names[@]}"
do
    echo "$file"
    if [[ "$file" == "code.py" ]]; then # do code.py as last
        codeflag=1
    else
      for ((i = 2; i <= $#; i++ )); do
        cp -v $file ${!i}
      done
    fi
done
if [[ "$codeflag" == "1" ]]; then # do code.py as last
  for ((i = 2; i <= $#; i++ )); do
    cp -v "code.py" ${!i}
  done
fi
