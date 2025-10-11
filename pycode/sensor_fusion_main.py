"""
SensorFusion Main Module

Main entry point for sensor fusion algorithms converted from MATLAB
"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import pandas as pd

class SensorFusion:
    """
    Main sensor fusion class - converted from MATLAB SF_Main.m
    """
    
    def __init__(self, fusion_type='9X_AGM'):
        """
        Initialize sensor fusion algorithm
        
        Args:
            fusion_type (str): Type of fusion ('9X_AGM' for 9-axis)
        """
        self.fusion_type = fusion_type
        self.state_initialized = False
        
        # Initialize global parameters (converted from MATLAB)
        self.DEG2RAD = np.pi / 180
        self.G2MPSECSQ = 9.8  # 1g = 9.8 m/s²
        self.B = 50.0  # μT
        self.NUM_GYRO_SAMP = 1
        self.EPSILON = 1e-6
        
        # Noise parameters (open source values)
        self.setup_noise_parameters()
    
    def setup_noise_parameters(self):
        """Setup noise parameters for the algorithm"""
        # Accelerometer parameters
        self.QvA = 2e-6  # accel measurement noise g²
        self.QwA = 1e-4  # accel drift g²
        self.Cacc = 0.5  # accel sensor noise time constant
        
        # Gyroscope parameters  
        self.QvG = 0.01  # gyro measurement noise (deg/s)²
        self.Qwb = 1e-9  # gyro offset drift (deg/s)²
        
        # Magnetometer parameters
        self.QvM = 0.1  # mag measurement noise μT²
        self.Qwd = 0.5  # mag disturbance drift μT²
        self.Cmd = 0.5  # mag disturbance noise time constant
    
    def load_data(self, input_file, method=2):
        """
        Load sensor data from file
        
        Args:
            input_file (str): Path to input data file
            method (int): Data loading method (1-6)
        
        Returns:
            dict: Loaded sensor data
        """
        if method == 2:
            # Excel file with separate sheets
            try:
                acc_data = pd.read_excel(input_file, sheet_name='Acc').values
                gyro_data = pd.read_excel(input_file, sheet_name='Gyro').values  
                mag_data = pd.read_excel(input_file, sheet_name='Mag').values
                quat_data = pd.read_excel(input_file, sheet_name='Quat').values
                orient_data = pd.read_excel(input_file, sheet_name='Orient').values
                
                return {
                    'acc': acc_data,
                    'gyro': gyro_data, 
                    'mag': mag_data,
                    'quat': quat_data,
                    'orient': orient_data
                }
            except Exception as e:
                print(f"Error loading data: {e}")
                return None
        
        # Add other methods as needed
        return None
    
    def update_fusion(self, acc_data, gyro_data, mag_data, timestamp):
        """
        Update sensor fusion with new sensor data
        
        Args:
            acc_data (array): Accelerometer data [x, y, z]
            gyro_data (array): Gyroscope data [x, y, z] 
            mag_data (array): Magnetometer data [x, y, z]
            timestamp (float): Timestamp
        
        Returns:
            dict: Updated state (quaternion, orientation angles)
        """
        # Placeholder for fusion algorithm
        # This would call the converted MATLAB functions
        
        # For now, return dummy data
        return {
            'quaternion': np.array([1.0, 0.0, 0.0, 0.0]),
            'phi': 0.0,    # Roll
            'theta': 0.0,  # Pitch  
            'psi': 0.0     # Yaw
        }
    
    def plot_results(self, results):
        """
        Plot sensor fusion results
        
        Args:
            results (dict): Fusion results to plot
        """
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        
        # Plot quaternions
        if 'quaternions' in results:
            q = results['quaternions']
            axes[0,0].plot(q[:, 0], label='q0')
            axes[0,0].plot(q[:, 1], label='q1')  
            axes[0,0].plot(q[:, 2], label='q2')
            axes[0,0].plot(q[:, 3], label='q3')
            axes[0,0].set_title('Quaternions')
            axes[0,0].legend()
            axes[0,0].grid(True)
        
        # Plot orientation angles
        if 'angles' in results:
            angles = results['angles']
            axes[0,1].plot(angles[:, 0], label='φ (Roll)')
            axes[0,1].plot(angles[:, 1], label='θ (Pitch)')
            axes[0,1].plot(angles[:, 2], label='ψ (Yaw)') 
            axes[0,1].set_title('Orientation Angles')
            axes[0,1].legend()
            axes[0,1].grid(True)
        
        plt.tight_layout()
        plt.show()

def main():
    """Main function for testing"""
    print("SensorFusion Python Module")
    print("Converted from MATLAB algorithms")
    
    # Example usage
    sf = SensorFusion()
    print(f"Fusion type: {sf.fusion_type}")
    print("Ready for sensor data processing")

if __name__ == "__main__":
    main()
