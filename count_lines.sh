find . -regextype posix-extended -type f -regex '.*\.(cpp|h|vsh|fsh|cu|pro)$' | grep -v google | xargs cat | wc -l
