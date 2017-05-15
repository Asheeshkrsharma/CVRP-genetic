g++ animate.cc -o animate
cat best-solution.txt | grep '>' | sed 's/-//g' | sed 's/>/\n/g' > tmp
./animate tmp `cat tmp | wc -l`
