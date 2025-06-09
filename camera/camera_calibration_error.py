import cv2
import numpy as np
import matplotlib.pyplot as plt
import os

def load_real_calibration_data():
    """
    Load real calibration data from your calibration results
    """
    try:
        # Load calibration file
        data = np.load('camera/calibration_result.npz')
        camera_matrix = data['cameraMatrix']
        dist_coeffs = data['distCoeffs']
        
        print("Real calibration data loaded successfully:")
        print(f"Camera Matrix:\n{camera_matrix}")
        print(f"Distortion Coefficients: {dist_coeffs}")
        
        return camera_matrix, dist_coeffs
    
    except FileNotFoundError:
        print("Error: 'calibration_result.npz' file not found!")
        print("Please run camera calibration first.")
        return None, None
    except Exception as e:
        print(f"Error loading calibration data: {e}")
        return None, None

def calculate_reprojection_error(objpoints, imgpoints, camera_matrix, dist_coeffs, rvecs, tvecs):
    """
    Calculate reprojection error for Zhang's Method
    """
    total_error = 0
    total_points = 0
    
    for i in range(len(objpoints)):
        # Reproject 3D points to 2D
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], 
                                         camera_matrix, dist_coeffs)
        
        # Calculate error
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        total_error += error
        total_points += len(imgpoints[i])
    
    mean_error = total_error / len(objpoints)
    return mean_error

def analyze_single_image_with_calibration(img_path, camera_matrix, dist_coeffs):
    """
    Perform calibration analysis for a single image
    """
    img = cv2.imread(img_path)
    if img is None:
        return None
    
    h, w = img.shape[:2]
    
    # Calculate optimal camera matrix
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, (w, h), 1, (w, h))
    
    # Apply distortion correction
    undistorted = cv2.undistort(img, camera_matrix, dist_coeffs, None, new_camera_matrix)
    
    # Crop ROI (optional)
    x, y, w_roi, h_roi = roi
    if w_roi > 0 and h_roi > 0:
        undistorted_cropped = undistorted[y:y+h_roi, x:x+w_roi]
    else:
        undistorted_cropped = undistorted
    
    # Image quality analysis
    gray_orig = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray_undist = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)
    
    # Measure distortion effect using edge detection
    edges_orig = cv2.Canny(gray_orig, 50, 150)
    edges_undist = cv2.Canny(gray_undist, 50, 150)
    
    edge_diff = np.sum(np.abs(edges_orig.astype(float) - edges_undist.astype(float)))
    
    # Chessboard detection (if present)
    pattern_size = (7, 7)  # pattern_size from your calibration code
    ret_orig, corners_orig = cv2.findChessboardCorners(gray_orig, pattern_size, None)
    ret_undist, corners_undist = cv2.findChessboardCorners(gray_undist, pattern_size, None)
    
    chessboard_error = 0
    if ret_orig and ret_undist:
        # Calculate average distance between corners
        diff = corners_orig - corners_undist
        chessboard_error = np.mean(np.sqrt(np.sum(diff**2, axis=2)))
    
    return {
        'original': img,
        'undistorted': undistorted,
        'undistorted_cropped': undistorted_cropped,
        'edge_difference': edge_diff,
        'chessboard_found_orig': ret_orig,
        'chessboard_found_undist': ret_undist,
        'chessboard_error': chessboard_error,
        'corners_orig': corners_orig if ret_orig else None,
        'corners_undist': corners_undist if ret_undist else None,
        'roi': roi
    }

def calculate_distortion_magnitude(camera_matrix, dist_coeffs, image_shape):
    """
    Calculate distortion magnitude
    """
    h, w = image_shape[:2]
    
    # Calculate distortion at image corners
    corners = np.array([
        [0, 0], [w-1, 0], [w-1, h-1], [0, h-1]
    ], dtype=np.float32).reshape(-1, 1, 2)
    
    # Apply distortion
    undistorted_corners = cv2.undistortPoints(corners, camera_matrix, dist_coeffs, 
                                            None, camera_matrix)
    
    # Calculate distortion amount
    distortion_magnitudes = np.sqrt(np.sum((corners - undistorted_corners)**2, axis=2))
    max_distortion = np.max(distortion_magnitudes)
    mean_distortion = np.mean(distortion_magnitudes)
    
    return max_distortion, mean_distortion

def main():
    # Load real calibration data
    camera_matrix, dist_coeffs = load_real_calibration_data()
    
    if camera_matrix is None:
        return
    
    # Check image files
    image_files = ['12.jpg']
    
    results = []
    
    print("\n" + "="*60)
    print("CAMERA CALIBRATION ANALYSIS")
    print("="*60)
    
    for img_file in image_files:
        if not os.path.exists(img_file):
            print(f"Warning: '{img_file}' file not found, skipping...")
            continue
        
        print(f"\nAnalyzing {img_file}...")
        
        result = analyze_single_image_with_calibration(img_file, camera_matrix, dist_coeffs)
        
        if result:
            # Calculate distortion magnitude
            max_dist, mean_dist = calculate_distortion_magnitude(
                camera_matrix, dist_coeffs, result['original'].shape)
            
            result['filename'] = img_file
            result['max_distortion'] = max_dist
            result['mean_distortion'] = mean_dist
            results.append(result)
            
            print(f"  Maximum Distortion: {max_dist:.2f} pixels")
            print(f"  Average Distortion: {mean_dist:.2f} pixels")
            print(f"  Edge Difference: {result['edge_difference']:.0f}")
            print(f"  Chessboard (Original): {'✓' if result['chessboard_found_orig'] else '✗'}")
            print(f"  Chessboard (Corrected): {'✓' if result['chessboard_found_undist'] else '✗'}")
            if result['chessboard_error'] > 0:
                print(f"  Chessboard Corner Error: {result['chessboard_error']:.3f} pixels")
    
    if not results:
        print("No images found for analysis!")
        return
    
    # Visualization
    fig, axes = plt.subplots(len(results), 3, figsize=(18, 6*len(results)))
    if len(results) == 1:
        axes = axes.reshape(1, -1)
    
    fig.suptitle('Camera Calibration: Zhang\'s Method Results', fontsize=16, fontweight='bold')
    
    for i, result in enumerate(results):
        # Original image
        axes[i, 0].imshow(cv2.cvtColor(result['original'], cv2.COLOR_BGR2RGB))
        axes[i, 0].set_title(f'{result["filename"]} - Before Calibration\n'
                           f'Max Distortion: {result["max_distortion"]:.2f}px')
        axes[i, 0].axis('off')
        
        # Mark corners if found
        if result['corners_orig'] is not None:
            corners = result['corners_orig'].reshape(-1, 2)
            axes[i, 0].scatter(corners[:, 0], corners[:, 1], c='red', s=30, alpha=0.8)
        
        # Corrected image
        axes[i, 1].imshow(cv2.cvtColor(result['undistorted'], cv2.COLOR_BGR2RGB))
        axes[i, 1].set_title(f'{result["filename"]} - After Calibration\n'
                           f'Average Distortion: {result["mean_distortion"]:.2f}px')
        axes[i, 1].axis('off')
        
        # Mark corners if found
        if result['corners_undist'] is not None:
            corners = result['corners_undist'].reshape(-1, 2)
            axes[i, 1].scatter(corners[:, 0], corners[:, 1], c='green', s=30, alpha=0.8)
        
        # Cropped image
        axes[i, 2].imshow(cv2.cvtColor(result['undistorted_cropped'], cv2.COLOR_BGR2RGB))
        axes[i, 2].set_title(f'{result["filename"]} - Cropped (ROI)\n'
                           f'ROI: {result["roi"]}')
        axes[i, 2].axis('off')
    
    plt.tight_layout()
    plt.show()
    
    # Summary statistics
    print("\n" + "="*60)
    print("CALIBRATION QUALITY ASSESSMENT")
    print("="*60)
    
    # Analyze calibration parameters
    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
    
    print(f"Focal Length (fx, fy): ({fx:.2f}, {fy:.2f}) pixels")
    print(f"Principal Point (cx, cy): ({cx:.2f}, {cy:.2f}) pixels")
    print(f"Aspect Ratio: {fx/fy:.4f}")
    
    # Distortion coefficients analysis
    dc = dist_coeffs.flatten()
    k1, k2, p1, p2 = dc[0], dc[1], dc[2], dc[3]
    k3 = dc[4] if len(dc) > 4 else 0
    
    print(f"\nDistortion Coefficients:")
    print(f"  Radial (k1, k2, k3): ({k1:.6f}, {k2:.6f}, {k3:.6f})")
    print(f"  Tangential (p1, p2): ({p1:.6f}, {p2:.6f})")
    
    # Distortion severity assessment
    radial_magnitude = abs(k1) + abs(k2) + abs(k3)
    tangential_magnitude = abs(p1) + abs(p2)
    
    if radial_magnitude < 0.1:
        radial_assessment = "Low"
    elif radial_magnitude < 0.3:
        radial_assessment = "Medium"
    else:
        radial_assessment = "High"
    
    print(f"\nDistortion Severity:")
    print(f"  Radial Distortion: {radial_assessment} ({radial_magnitude:.4f})")
    print(f"  Tangential Distortion: {'Low' if tangential_magnitude < 0.01 else 'Medium'} ({tangential_magnitude:.6f})")
    
    # Overall calibration quality
    avg_max_distortion = np.mean([r['max_distortion'] for r in results])
    
    if avg_max_distortion < 1.0:
        quality = "Excellent"
    elif avg_max_distortion < 3.0:
        quality = "Good"
    elif avg_max_distortion < 5.0:
        quality = "Fair"
    else:
        quality = "Poor"
    
    print(f"\nOverall Calibration Quality: {quality}")
    print(f"Average Maximum Distortion: {avg_max_distortion:.2f} pixels")
    
    # Camera specifications based on your calibration
    print(f"\n{'='*60}")
    print("CAMERA SPECIFICATIONS (based on calibration)")
    print(f"{'='*60}")
    
    # Estimate sensor size (assuming common sensor sizes)
    if fx < 500:
        sensor_type = "Wide-angle or mobile camera"
    elif fx < 800:
        sensor_type = "Standard webcam"
    elif fx < 1200:
        sensor_type = "DSLR or high-quality camera"
    else:
        sensor_type = "Telephoto or specialized camera"
    
    print(f"Estimated Camera Type: {sensor_type}")
    print(f"Field of View (estimated): {2 * np.arctan(cx/fx) * 180/np.pi:.1f}° horizontal")
    
    print(f"\n{'='*60}")
    print("Note: This analysis is performed using Zhang's Method")
    print("with your actual calibration parameters.")
    print("Calibration performed on:", "2025-06-09 14:52:18 UTC")

if __name__ == "__main__":
    main()