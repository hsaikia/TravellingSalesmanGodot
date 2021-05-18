extends Node2D

var rng = RandomNumberGenerator.new()
var node_scene = preload("res://scenes/ACONode.tscn")
var edge_scene = preload("res://scenes/ACOPath.tscn")
var nodes = []
var edges = []
var num_nodes = 20
var node_radius = 50
var edge_width = 100
var edge_scale = 0.1
var weights = []
var distances = []
var running = false
var best_len = 1000000000
var alpha = 1.0 # pheromone weight
var beta = 2.0 # greedy weight

func run_aco_batch(batch_size: int):
	running = false
	# Evaporate
	for i in range(num_nodes):
		for j in range(num_nodes):
			weights[i][j] *= 0.999

	var new_weights = weights.duplicate(true)
	# Run the whole batch
	for _i in range(batch_size):
		var ret = get_random_path_from(rng.randi_range(0, num_nodes - 1))
		var path = ret[0]
		var l = ret[1]
		
		if l < best_len:
			best_len = l
		var diff = l - best_len + 0.05
		var w = 0.01 / diff
		for i in range(num_nodes + 1):
			var idx1 = path[i % num_nodes]
			var idx2 = path[(i + 1) % num_nodes]
			new_weights[idx1][idx2] += w
			new_weights[idx2][idx1] += w

	# Update the weights after normalizing
	for i in range(num_nodes):
		var n_sum = 0.0
		for j in range(num_nodes):
			if i == j:
				continue
			n_sum += new_weights[i][j]
		for j in range(num_nodes):
			# multiplying by 2 since every node has two neighbors eventually
			weights[i][j] = 2 * new_weights[i][j] / n_sum
	add_edges()		
	draw_stuff()
	running = true
	return best_len

func get_transition_probability(idx1 : int, idx2 : int) -> float:
	return pow(weights[idx1][idx2], alpha) * pow(distances[idx1][idx2], -beta)

func get_random_path_from(idx : int):
	var path = []
	var dist = 0.0
	path.append(idx)
	var curr_idx = idx
	while len(path) < num_nodes:
		var n_sum = 0.0;
		var possible_next = []
		for n in range(num_nodes):
			if path.has(n): # already visited
				continue
			n_sum += get_transition_probability(curr_idx, n)
			possible_next.append(n)
		var r = rng.randf_range(0.0, n_sum)
		var x = 0.0
		for nn in possible_next:
			x += get_transition_probability(curr_idx, nn)
			if r <= x:
				dist += distances[curr_idx][nn]
				curr_idx = nn
				path.append(nn)
				break
	dist += distances[curr_idx][idx]
	return [path, dist]

func add_node(x, y):
	var node_var = node_scene.instance()
	node_var.set_position(Vector2(x, y))
	nodes.append(node_var)

func force(me : Vector2, other : Vector2, attractive : bool) -> Vector2:
	# constant attraction
	var ka = 0.1 * num_nodes
	# repulsion
	var kr = -10000
	var eps = 0.001
	var ret = 1.0 / eps
	var dist_squared = (me - other).length_squared()
	#print("Dist ", dist_squared)
	if dist_squared > eps :
		ret = 1.0 / dist_squared
	if attractive:
		ret = ka
	else:
		ret *= kr
	var ret_vec = ret * (other - me).normalized()
	return ret_vec

# use a force directed layout
func spread_nodes(num_iter : int) -> void:
	# all nodes are attracted to the center
	# and repelled by each other
	for _i in range(num_iter):
		var center = Vector2(0, 0)
		for node in nodes:
			center += node.position
		center /= num_nodes
		var new_pos = []
		for i in range(num_nodes):
			new_pos.append(nodes[i].position)
			for j in range(num_nodes):
				if i == j:
					continue
				new_pos[i] += force(nodes[i].position, nodes[j].position, false)
			new_pos[i] += force(nodes[i].position, center, true)
		for i in range(num_nodes):
			nodes[i].set_position(new_pos[i])
	add_weights()
	add_edges()		
	draw_stuff()

func add_edges():
	edges = []
	for i in range(num_nodes):
		for j in range(i + 1, num_nodes):
			add_edge(i, j)

func add_weights():
	# clear old and add new edges
	weights = []
	distances = []
	for i in range(num_nodes):
		var weight_i = []
		var distance_i = []
		for j in range(num_nodes):
			if i == j:
				weight_i.append(0.0)
				distance_i.append(0.0)
				continue
			elif j < i:
				weight_i.append(1.0)
				distance_i.append(distances[j][i])
				continue
			weight_i.append(1.0)
			distance_i.append((nodes[i].position - nodes[j].position).length())
		weights.append(weight_i)
		distances.append(distance_i)

func add_edge(i1, i2):
	# edge weight
	var w = weights[i1][i2]
	# effective edge scale
	var eff_scale = w * edge_scale
	# displacement vector
	var disp = nodes[i2].position - nodes[i1].position
	# heading of displacement vector
	var rot_angle = atan2(disp.y, disp.x)
	# edge center displacement (since is edge is centered at top-left)
	var edge_center_disp = Vector2(0, - edge_width * eff_scale/2)
	# applying rotation
	var edge_rot_disp = edge_center_disp.rotated(rot_angle)
	var edge_var = edge_scene.instance()
	edge_var.set_rotation(rot_angle)
	edge_var.set_position(Vector2(nodes[i1].position.x, nodes[i1].position.y) + edge_rot_disp)
	edge_var.set_scale(Vector2(eff_scale, eff_scale))
	var eps = 0.001
	if eff_scale > eps:
		edge_var.set_size(Vector2(disp.length() / eff_scale, edge_width))
	else:
		edge_var.set_size(Vector2(1.0 / eps, edge_width))
	edges.append(edge_var)

func draw_stuff():
	for ch in self.get_children():
		self.remove_child(ch)
	for edge in edges:
		self.add_child(edge)
	for node in nodes:
		self.add_child(node)

func _ready():
	rng.seed = OS.get_ticks_msec()
	generate()

func generate():
	best_len = 1000000000
	nodes = []
	edges = []
	var WS = get_viewport().get_visible_rect().size
	var L = int(WS.x) - 2 * node_radius
	var W = int(WS.y) - 2 * node_radius
	for _n in range(num_nodes):
		add_node(rng.randi()%L + node_radius, rng.randi()%W + node_radius)
	# spread nodes
	spread_nodes(40)
	draw_stuff()
	
func _process(_delta):
	if Input.is_action_just_released("generate"):
		generate()
	if Input.is_action_just_released("spread"):
		spread_nodes(5)
	if Input.is_action_just_released("add_node"):
		num_nodes += 1
		generate()
	if Input.is_action_just_released("delete_node"):
		num_nodes = max(num_nodes - 1, 3)
		generate()
	if Input.is_action_just_released("optimize"):
		running = !running
	if running:
		best_len = run_aco_batch(20)
