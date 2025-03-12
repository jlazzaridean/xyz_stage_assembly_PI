import pi_custom_xyz as xyz

print('Greetings! You are about to home the XYZ stage.')
print('This is a DANGEROUS OPERATION that can crash the objective.')
print('Please confirm that the SAMPLE IS REMOVED FROM THE HOLDER.')
print('Please confirm that the STAGE INSERT IS REMOVED.')
print('When you have determined it is safe, please type YES below.')


s = input('\n\nType YES and hit return to begin homing operation: ')

if s == 'YES' or s == 'yes':
    print('Beginning stage homing...')
    xyz_stage = xyz.PI_Custom_XYZ_Stage()
    xyz_stage.reset_to_default_velocity_acceleration()
    xyz_stage.home_all_axes()
    xyz_stage.close(disable_motors=False)
    print('Stage homing complete.')
else:
    print('Stage homing aborted. Exiting.')
