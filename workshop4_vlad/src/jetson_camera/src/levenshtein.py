def print_matrix(m):
    for i in range(len(m)):
        print(m[i])

# calculates the "distance" between two strings using levenshtein distance algorithm
def levenshtein(a, b):
    
    len_a = len(a)
    len_b = len(b)
    
    if len(a) == 0:
        return len(b)
    
    if len(b) == 0:
        return len(a)
    
    d = [([0] * (len_a + 1)) for i in range(len_b + 1)]

    for j in range(1, len_a+1):
        d[0][j] = j
        
    for i in range(1, len_b+1):
        d[i][0] = i

    for i in range(1, len_b + 1):
        for j in range(1, len_a + 1):
            sub_cost = 0

            if a[j - 1] != b[i - 1]:
                sub_cost = 1
                
            d[i][j] = min(d[i-1][j] + 1, # deletion
                        d[i][j - 1] + 1, # insertion
                        d[i-1][j-1] + sub_cost # substitution
                          )
    
    # print_matrix(d)
    return d[len_b][len_a]
            
    
# res = levenshetin("abcde", "abde")
# print(res)