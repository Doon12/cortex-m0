Program_name="bubble_sort"
make
gcc B2H_CONVERTER.c -o B2H_CONVERTER
./B2H_CONVERTER ${Program_name}.bin ${Program_name}.hex
cp ${Program_name}.hex ../
