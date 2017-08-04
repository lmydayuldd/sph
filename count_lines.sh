find . -regextype posix-extended -type f -regex '.*\.(cpp|h|vsh|fsh)$' | grep -v google | xargs cat | wc -l
