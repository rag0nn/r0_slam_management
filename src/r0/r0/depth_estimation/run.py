"""Compute depth maps for images in the input folder.
"""
import torch

import cv2
import time

import numpy as np

from .midas.model_loader import default_models, load_model




class MidasEstimator():

    
    def __init__(self,model_path='/home/rag0n/Desktop/lab/r0_workspace/src/r0/r0/depth_estimation/midas_v21_small_256.pt', model_type="midas_v21_small_256", optimize=False, side=False, height=None,
             square=False, grayscale=False):
        self.first_execution = True
        print("Initialize")
        # set torch options

        torch.backends.cudnn.enabled = True
        torch.backends.cudnn.benchmark = True

        # select device
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print("Device: %s" % self.device)

        self.model_type = model_type
        self.optimize = optimize
        self.side = side
        self.height = height
        self.square = square
        self.grayscale = grayscale
        self.model, self.transform, self.net_w, self.net_h = load_model(self.device, model_path, model_type, optimize, height, square)

        print("Model Successfully Loaded - Model başarıyla yüklendi")


    def process(self,device, model, model_type, image, input_size, target_size, optimize, use_camera):
        """
        Run the inference and interpolate.

        Args:
            device (torch.device): the torch device used
            model: the model used for inference
            model_type: the type of the model
            image: the image fed into the neural network
            input_size: the size (width, height) of the neural network input (for OpenVINO)
            target_size: the size (width, height) the neural network output is interpolated to
            optimize: optimize the model to half-floats on CUDA?
            use_camera: is the camera used?

        Returns:
            the prediction
        """

        if "openvino" in model_type:
            if self.first_execution or not use_camera:
                print(f"    Input resized to {input_size[0]}x{input_size[1]} before entering the encoder")
                self.first_execution = False

            sample = [np.reshape(image, (1, 3, *input_size))]
            prediction = model(sample)[model.output(0)][0]
            prediction = cv2.resize(prediction, dsize=target_size,
                                    interpolation=cv2.INTER_CUBIC)
        else:
            sample = torch.from_numpy(image).to(device).unsqueeze(0)

            if optimize and device == torch.device("cuda"):
                if self.first_execution:
                    print("  Optimization to half-floats activated. Use with caution, because models like Swin require\n"
                        "  float precision to work properly and may yield non-finite depth values to some extent for\n"
                        "  half-floats.")
                sample = sample.to(memory_format=torch.channels_last)
                sample = sample.half()

            if self.first_execution or not use_camera:
                height, width = sample.shape[2:]
                print(f"    Input resized to {width}x{height} before entering the encoder")
                self.first_execution = False

            try:
                prediction = model.forward(sample)
                prediction = (
                    torch.nn.functional.interpolate(
                        prediction.unsqueeze(1),
                        size=target_size[::-1],
                        mode="bicubic",
                        align_corners=False,
                    )
                    .squeeze()
                    .cpu()
                    .numpy()
                )
            except Exception as e:
                print(f"Model not estimated this image {e}")
        return prediction


    def create_side_by_side(self,image, depth, grayscale):
        """
        Take an RGB image and depth map and place them side by side. This includes a proper normalization of the depth map
        for better visibility.

        Args:
            image: the RGB image
            depth: the depth map
            grayscale: use a grayscale colormap?

        Returns:
            the image and depth map place side by side
        """
        depth_min = depth.min()
        depth_max = depth.max()
        normalized_depth = 255 * (depth - depth_min) / (depth_max - depth_min)
        normalized_depth *= 3

        right_side = np.repeat(np.expand_dims(normalized_depth, 2), 3, axis=2) / 3
        if not grayscale:
            right_side = cv2.applyColorMap(np.uint8(right_side), cv2.COLORMAP_INFERNO)

        if image is None:
            return right_side
        else:
            return np.concatenate((image, right_side), axis=1)




    def estimate(self,input_image,):
        """Run MonoDepthNN to compute depth maps.

        Args:
            input_path (str): path to input folder
            output_path (str): path to output folder
            model_path (str): path to saved model
            model_type (str): the model type
            optimize (bool): optimize the model to half-floats on CUDA?
            side (bool): RGB and depth side by side in output images?
            height (int): inference encoder image height
            square (bool): resize to a square resolution?
            grayscale (bool): use a grayscale colormap?
        """



        

        with torch.no_grad():
            time_start = time.time()
            frame_index = 0
            fps = 1
            if input_image is not None:
                original_image_rgb = np.flip(input_image, 2)  # in [0, 255] (flip required to get RGB)
                image = self.transform({"image": original_image_rgb/255})["image"]

                prediction = self.process(self.device, self.model, self.model_type, image, (self.net_w, self.net_h),
                                        original_image_rgb.shape[1::-1], self.optimize, True)

                original_image_bgr = np.flip(original_image_rgb, 2) if self.side else None
                content = self.create_side_by_side(original_image_bgr, prediction, self.grayscale)
                #cv2.imshow('MiDaS Depth Estimation - Press Escape to close window ', content/255)

                alpha = 0.1
                if time.time()-time_start > 0:
                    fps = (1 - alpha) * fps + alpha * 1 / (time.time()-time_start)  # exponential moving average
                    time_start = time.time()
                #print(f"\rFPS: {round(fps,2)}", end="")
                frame_index += 1
            print()

        print("Depth Estimated")
        return content