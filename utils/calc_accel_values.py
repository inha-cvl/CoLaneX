import sys

def main():
    txt_path = sys.argv[1]
    bp = [0, 2.7, 6.9, 11]
    a_sum = [0, 0, 0, 0]
    a_cnt = [0, 0, 0, 0]

    with open(txt_path, 'r') as f:
        for line in f:
            v, a = map(float, line.split())
            
            for i in range(4):
                if bp[i] <= v < bp[i+1] if i < 3 else bp[i] <= v:
                    a_sum[i] += a
                    a_cnt[i] += 1

    avg_a = [s/c if c > 0 else 0 for s, c in zip(a_sum, a_cnt)]
    print(avg_a)

if __name__ == '__main__':
    main()