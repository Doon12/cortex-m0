/*********************************************************
 *                                                       *
 *  EE511 Midterm Project                                *
 *                                                       *
 *  Example test code                                    *
 *                                                       *
 *  April, 2018                                          *
 *                                                       *
 *  Written by : Daesung Kim (dskim@ics.kaist.ac.kr)     *
 *                                                       *
 *********************************************************/

int test0 __attribute__ ((section ("VERIFY0")));
int test1 __attribute__ ((section ("VERIFY1")));
int test2 __attribute__ ((section ("VERIFY2")));
int test3 __attribute__ ((section ("VERIFY3")));
int test4 __attribute__ ((section ("VERIFY4")));
int test5 __attribute__ ((section ("VERIFY5")));
int test6 __attribute__ ((section ("VERIFY6")));
int test7 __attribute__ ((section ("VERIFY7")));
int test8 __attribute__ ((section ("VERIFY8")));
int test9 __attribute__ ((section ("VERIFY9")));

int main(void)
{
  int array[10] = {8, 7, 1, 2, 6, 9, 0, 3, 5, -1};
  int n = 10, c, d, swap;

  for (c = 0 ; c < ( n - 1 ); c++) {
    for (d = 0 ; d < n - c - 1; d++) {
      if (array[d] > array[d + 1]) {
        swap = array[d];
        array[d] = array[d + 1];
        array[d + 1] = swap;
      }
    }
  }

  test0 = array[0];
  test1 = array[1];
  test2 = array[2];
  test3 = array[3];
  test4 = array[4];
  test5 = array[5];
  test6 = array[6];
  test7 = array[7];
  test8 = array[8];
  test9 = array[9];

  return 0;
}

