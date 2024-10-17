from setuptools import setup, find_packages

setup(
    name='br_hand_package',
    version='1.0.0',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    install_requires=['rospy', 
                      'std_msgs', 
                      'message_generation'],
    entry_poinst={
        'consol_scripts': [
            'hand_node = br_hand.hand_node.main',
            'gui_node = br_hand.gui_node.main',
            'node_manager = br_hand.node_manager.main'
        ],
    },
)
