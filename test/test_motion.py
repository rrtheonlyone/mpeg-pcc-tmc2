from collections import defaultdict
from pprint import pprint

def diff(v1, v2):
	dx = v1[0] - v2[0]
	dy = v1[1] - v2[1]
	dz = v1[2] - v2[2]
	return [dx, dy, dz]

def dist(v1):
	return (v1[0]*v1[0] + v1[1]*v1[1] + v1[2]*v1[2]) ** (0.5)

def calc_diff(k):
	e1 = dist(diff(k[0], k[1]))
	e2 = dist(diff(k[1], k[2]))
	return (e1, e2)

def main1():
	mv_data = []
	rv_data = []

	for j in range(1, 4):
		raw_data = open(f"motion_stream{j}.txt", "r")
		md = defaultdict(dict) 
		md_rev = defaultdict(dict)

		for raw_vector in raw_data:
			raw_vector = raw_vector.strip()
			dt = [int(x) for x in raw_vector.split(" ")]
			md[dt[0]][dt[1]] = [dt[2], dt[3], dt[4]]
			md_rev[dt[1]][dt[0]] = [dt[2], dt[3], dt[4]]

		mv_data.append(md)
		rv_data.append(md_rev)

	#let's calculate diff from MV1 and MV2
	csv_file = ["p1,p1_r,dx,dy,dz,p2,p2_r,dx1,dy1,dz1,dist"]

	curr_p = 1
	ref_p = curr_p + 1

	for curr, delta in mv_data[curr_p].items():
		p1 = curr
		p1_r = list(delta.items())[0][0]
		dx,dy,dz = list(delta.items())[0][1]

		p2_r = p1 
		for p2, delta1 in rv_data[ref_p][p2_r].items():
			dx1, dy1, dz1 = delta1
			csv_row = [
				p1, p1_r, dx, dy, dz,
				p2, p2_r, dx1, dy1, dz1,
				dist([dx - dx1, dy - dy1, dz - dz1])
			]
			csv_file.append(",".join([str(x) for x in csv_row]))

	print("\n".join(csv_file))

def main():
	raw_data = open(f"m3_original.txt", "r")
	md = defaultdict(int) 
	for raw_vector in raw_data:
		raw_vector = raw_vector.strip()
		dt = [int(x) for x in raw_vector.split(" ")]	
		md[(dt[0], dt[1], dt[2])] += 1

	for k,v in md.items():
		if v <= 1: 
			continue 
		print(k)



if __name__ == "__main__":
	main1()
