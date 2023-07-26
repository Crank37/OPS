

Tag_Poses = {}

new_name = "tag1"


if new_name  not in Tag_Poses:
    Tag_Poses.update({new_name : {"x_t" : 1, "y_t" : 2, "z_t" : 3 , "w_o" : 1, "x_o" : 2, "y_o" : 3, "z_o" : 4}})
    print("geklappt1")

new_name = "tag2"

if new_name  not in Tag_Poses:
    Tag_Poses.update({new_name : {"x_t" : 4, "y_t" : 2, "z_t" : 3 , "w_o" : 1, "x_o" : 2, "y_o" : 3, "z_o" : 4}})
    print("geklappt2")


#fischt sich alle INformationen aus Dictionary
for i in Tag_Poses.keys():        
    
    #Dict Values in Liste abgewandelt
    abs_coordinateApril_value = list(Tag_Poses[i].values())

    print("----------------------------------------")
    print(" Name : " + str(i))
    print("Position: ")
    print("  X: " + str(abs_coordinateApril_value[0]))
    print("  Y: " + str(abs_coordinateApril_value[1]))
    # print("  Z: " + str(trans.transform.translation.z))
    # print("Orientation: ")
    # print("  X: " + str(trans.transform.rotation.x))
    # print("  Y: " + str(trans.transform.rotation.y))
    # print("  Z: " + str(trans.transform.rotation.z))
    # print("  W: " + str(trans.transform.rotation.w))