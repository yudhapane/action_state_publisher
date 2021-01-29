import sys, os
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np
from collections import defaultdict 

# plt.style.use('classic')

ex_results = ['SAT.csv', 'PRP.csv', 'SP$_{FD}$.csv', 'SP$_{FF}$.csv', 'SP$_{M}$.csv', 'SP$_{LPG-TD}$.csv']

COLOR = { 0 : 'navy',
          1 : 'skyblue',
          2 : 'darkcyan',
          3 : 'black' }

# load a list of domains (directories)
domains = sorted([d for d in os.listdir(r'.') if os.path.isdir(d) and not d == 'output'])

print('domains:', domains, len(domains))

solved = defaultdict(list)
for domain in domains:
    for result in os.listdir(domain):
        if result in ex_results: continue

        df = pd.read_csv(domain+'/'+result,header=0).fillna(0)
        solved[os.path.splitext(os.path.basename(result))[0]].append(\
                sum(df['solvable'].values.tolist()))

print('solved problems:')
for i, key in enumerate(solved):
    print(i, key, solved[key], len(solved[key]))

x = np.arange(len(domains))  # the label locations
width = 0.44  # the width of the bars

fig, ax = plt.subplots(figsize=(10, 5))
# fig, ax = plt.subplots(figsize=(14, 7))

rects = []

i = 0
for key in solved:
    if len(solved[key]) == len(domains):
        rects.append( ax.bar(x + width*i, solved[key], width, label=key, color=COLOR[i%len(COLOR)]) )
        i += 1

# Add some text for labels, title and custom x-axis tick labels, etc.
ax.set_ylabel('# solved problems')
ax.set_xticks(x + width)
ax.set_xticklabels(domains, rotation=40)
ax.legend(loc='upper left', frameon=False)

def autolabel(rects):
    """Attach a text label above each bar in *rects*, displaying its height."""
    for rect in rects:
        height = rect.get_height()
        ax.annotate('{:g}'.format(height),
                    xy=(rect.get_x() + rect.get_width() / 2, height),
                    xytext=(0, 3),  # 3 points vertical offset
                    textcoords="offset points",
                    ha='center', va='bottom', fontsize=10)
for rect in rects:
    autolabel(rect)

# fig.tight_layout(pad=.5)
fig.savefig('plot_solved_all.pdf', bbox_inches='tight')

# plt.show()

# crop the output pdf file
os.system('pdfcrop plot_solved_all.pdf plot_solved_all.pdf')
