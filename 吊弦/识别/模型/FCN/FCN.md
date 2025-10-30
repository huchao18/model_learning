# FCN

# [Kaggle地址](https://www.kaggle.com/code/hushichao111/fcn-dropper/edit)

## 数据集定义

```python
import os
import torch
from torch.utils.data import Dataset
from PIL import Image
import numpy as np
from torch.utils.data import DataLoader
from torchvision import transforms
import albumentations as A
from albumentations.pytorch import ToTensorV2
class dropperDataset(Dataset):
    def __init__(self, image_dir, mask_dir, transform=None):
        self.image_dir = image_dir
        self.mask_dir = mask_dir
        self.transform = transform
        self.image_list = sorted([f for f in os.listdir(image_dir)])

    def __len__(self):
        return len(self.image_list)

    def __getitem__(self, idx):
        image_path = os.path.join(self.image_dir, self.image_list[idx])
        mask_name = os.path.splitext(self.image_list[idx])[0] + ".npy"
        mask_path = os.path.join(self.mask_dir, mask_name)

        # 读取图像
        image = Image.open(image_path).convert("RGB")
        mask_np = np.load(mask_path).astype(np.int64)
        mask = torch.from_numpy(mask_np)

        # 横竖屏统一
        w, h = image.size
        if h > w:
            image = image.rotate(90, expand=True)
            mask = mask.transpose(0,1)
            mask = torch.flip(mask, dims=[0])

        # 转 numpy 给 Albumentations
        image_np = np.array(image)
        mask_np = mask.numpy()

        if self.transform:
            augmented = self.transform(image=image_np, mask=mask_np)
            image = augmented['image']
            mask = augmented['mask']

        return image, mask
```

## 数据集划分

```python
image_dir='/kaggle/input/dropper/Dropper/JPEGImages'
mask_dir='/kaggle/input/dropper/Dropper/SegmentationClass'
# 定义训练增强
transform = A.Compose([
    A.HorizontalFlip(p=0.5),           # 随机水平翻转
    A.RandomCrop(width=1920, height=1056), # 随机裁剪
    A.Normalize(mean=(0.5,0.5,0.5), std=(0.5,0.5,0.5)), # 图像归一化
    ToTensorV2()                        # 转为 tensor
])
dataset=dropperDataset(image_dir=image_dir,
                       mask_dir=mask_dir,transform=transform)
#划分数据集
from sklearn.model_selection import train_test_split
from torch.utils.data import Subset
indices=np.arange(len(dataset))
train_idx,tem_idx=train_test_split(indices,test_size=0.3,random_state=42)
val_idx,test_idx=train_test_split(tem_idx,test_size=0.33,random_state=42)
#用subset
train_dataset=Subset(dataset,train_idx)
val_dataset=Subset(dataset,val_idx)
test_dataset=Subset(dataset,test_idx)
print("训练集：{},验证集：{},测试集：{}".format(len(train_dataset),len(val_dataset),len(test_dataset)))
```

## FCN模型定义

```python
# -*- coding: utf-8 -*-

from __future__ import print_function

import torch
import torch.nn as nn
import torch.optim as optim
from torchvision import models
from torchvision.models.vgg import VGG


class FCN32s(nn.Module):

    def __init__(self, pretrained_net, n_class):
        super().__init__()
        self.n_class = n_class
        self.pretrained_net = pretrained_net
        self.relu    = nn.ReLU(inplace=True)
        self.deconv1 = nn.ConvTranspose2d(512, 512, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn1     = nn.BatchNorm2d(512)
        self.deconv2 = nn.ConvTranspose2d(512, 256, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn2     = nn.BatchNorm2d(256)
        self.deconv3 = nn.ConvTranspose2d(256, 128, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn3     = nn.BatchNorm2d(128)
        self.deconv4 = nn.ConvTranspose2d(128, 64, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn4     = nn.BatchNorm2d(64)
        self.deconv5 = nn.ConvTranspose2d(64, 32, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn5     = nn.BatchNorm2d(32)
        self.classifier = nn.Conv2d(32, n_class, kernel_size=1)

    def forward(self, x):
        output = self.pretrained_net(x)
        x5 = output['x5']  # size=(N, 512, x.H/32, x.W/32)

        score = self.bn1(self.relu(self.deconv1(x5)))     # size=(N, 512, x.H/16, x.W/16)
        score = self.bn2(self.relu(self.deconv2(score)))  # size=(N, 256, x.H/8, x.W/8)
        score = self.bn3(self.relu(self.deconv3(score)))  # size=(N, 128, x.H/4, x.W/4)
        score = self.bn4(self.relu(self.deconv4(score)))  # size=(N, 64, x.H/2, x.W/2)
        score = self.bn5(self.relu(self.deconv5(score)))  # size=(N, 32, x.H, x.W)
        score = self.classifier(score)                    # size=(N, n_class, x.H/1, x.W/1)

        return score  # size=(N, n_class, x.H/1, x.W/1)


class FCN16s(nn.Module):

    def __init__(self, pretrained_net, n_class):
        super().__init__()
        self.n_class = n_class
        self.pretrained_net = pretrained_net
        self.relu    = nn.ReLU(inplace=True)
        self.deconv1 = nn.ConvTranspose2d(512, 512, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn1     = nn.BatchNorm2d(512)
        self.deconv2 = nn.ConvTranspose2d(512, 256, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn2     = nn.BatchNorm2d(256)
        self.deconv3 = nn.ConvTranspose2d(256, 128, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn3     = nn.BatchNorm2d(128)
        self.deconv4 = nn.ConvTranspose2d(128, 64, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn4     = nn.BatchNorm2d(64)
        self.deconv5 = nn.ConvTranspose2d(64, 32, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn5     = nn.BatchNorm2d(32)
        self.classifier = nn.Conv2d(32, n_class, kernel_size=1)

    def forward(self, x):
        output = self.pretrained_net(x)
        x5 = output['x5']  # size=(N, 512, x.H/32, x.W/32)
        x4 = output['x4']  # size=(N, 512, x.H/16, x.W/16)

        score = self.relu(self.deconv1(x5))               # size=(N, 512, x.H/16, x.W/16)
        score = self.bn1(score + x4)                      # element-wise add, size=(N, 512, x.H/16, x.W/16)
        score = self.bn2(self.relu(self.deconv2(score)))  # size=(N, 256, x.H/8, x.W/8)
        score = self.bn3(self.relu(self.deconv3(score)))  # size=(N, 128, x.H/4, x.W/4)
        score = self.bn4(self.relu(self.deconv4(score)))  # size=(N, 64, x.H/2, x.W/2)
        score = self.bn5(self.relu(self.deconv5(score)))  # size=(N, 32, x.H, x.W)
        score = self.classifier(score)                    # size=(N, n_class, x.H/1, x.W/1)

        return score  # size=(N, n_class, x.H/1, x.W/1)


class FCN8s(nn.Module):

    def __init__(self, pretrained_net, n_class):
        super().__init__()
        self.n_class = n_class
        self.pretrained_net = pretrained_net
        self.relu    = nn.ReLU(inplace=True)
        self.deconv1 = nn.ConvTranspose2d(512, 512, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn1     = nn.BatchNorm2d(512)
        self.deconv2 = nn.ConvTranspose2d(512, 256, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn2     = nn.BatchNorm2d(256)
        self.deconv3 = nn.ConvTranspose2d(256, 128, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn3     = nn.BatchNorm2d(128)
        self.deconv4 = nn.ConvTranspose2d(128, 64, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn4     = nn.BatchNorm2d(64)
        self.deconv5 = nn.ConvTranspose2d(64, 32, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn5     = nn.BatchNorm2d(32)
        self.classifier = nn.Conv2d(32, n_class, kernel_size=1)

    def forward(self, x):
        output = self.pretrained_net(x)
        x5 = output['x5']  # size=(N, 512, x.H/32, x.W/32)
        x4 = output['x4']  # size=(N, 512, x.H/16, x.W/16)
        x3 = output['x3']  # size=(N, 256, x.H/8,  x.W/8)

        score = self.relu(self.deconv1(x5))               # size=(N, 512, x.H/16, x.W/16)
        score = self.bn1(score + x4)                      # element-wise add, size=(N, 512, x.H/16, x.W/16)
        score = self.relu(self.deconv2(score))            # size=(N, 256, x.H/8, x.W/8)
        score = self.bn2(score + x3)                      # element-wise add, size=(N, 256, x.H/8, x.W/8)
        score = self.bn3(self.relu(self.deconv3(score)))  # size=(N, 128, x.H/4, x.W/4)
        score = self.bn4(self.relu(self.deconv4(score)))  # size=(N, 64, x.H/2, x.W/2)
        score = self.bn5(self.relu(self.deconv5(score)))  # size=(N, 32, x.H, x.W)
        score = self.classifier(score)                    # size=(N, n_class, x.H/1, x.W/1)

        return score  # size=(N, n_class, x.H/1, x.W/1)


class FCNs(nn.Module):

    def __init__(self, pretrained_net, n_class):
        super().__init__()
        self.n_class = n_class
        self.pretrained_net = pretrained_net
        self.relu    = nn.ReLU(inplace=True)
        self.deconv1 = nn.ConvTranspose2d(512, 512, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn1     = nn.BatchNorm2d(512)
        self.deconv2 = nn.ConvTranspose2d(512, 256, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn2     = nn.BatchNorm2d(256)
        self.deconv3 = nn.ConvTranspose2d(256, 128, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn3     = nn.BatchNorm2d(128)
        self.deconv4 = nn.ConvTranspose2d(128, 64, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn4     = nn.BatchNorm2d(64)
        self.deconv5 = nn.ConvTranspose2d(64, 32, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn5     = nn.BatchNorm2d(32)
        self.classifier = nn.Conv2d(32, n_class, kernel_size=1)

    def forward(self, x):
        output = self.pretrained_net(x)
        x5 = output['x5']  # size=(N, 512, x.H/32, x.W/32)
        x4 = output['x4']  # size=(N, 512, x.H/16, x.W/16)
        x3 = output['x3']  # size=(N, 256, x.H/8,  x.W/8)
        x2 = output['x2']  # size=(N, 128, x.H/4,  x.W/4)
        x1 = output['x1']  # size=(N, 64, x.H/2,  x.W/2)

        score = self.bn1(self.relu(self.deconv1(x5)))     # size=(N, 512, x.H/16, x.W/16)
        score = score + x4                                # element-wise add, size=(N, 512, x.H/16, x.W/16)
        score = self.bn2(self.relu(self.deconv2(score)))  # size=(N, 256, x.H/8, x.W/8)
        score = score + x3                                # element-wise add, size=(N, 256, x.H/8, x.W/8)
        score = self.bn3(self.relu(self.deconv3(score)))  # size=(N, 128, x.H/4, x.W/4)
        score = score + x2                                # element-wise add, size=(N, 128, x.H/4, x.W/4)
        score = self.bn4(self.relu(self.deconv4(score)))  # size=(N, 64, x.H/2, x.W/2)
        score = score + x1                                # element-wise add, size=(N, 64, x.H/2, x.W/2)
        score = self.bn5(self.relu(self.deconv5(score)))  # size=(N, 32, x.H, x.W)
        score = self.classifier(score)                    # size=(N, n_class, x.H/1, x.W/1)

        return score  # size=(N, n_class, x.H/1, x.W/1)


class VGGNet(VGG):
    def __init__(self, pretrained=True, model='vgg16', requires_grad=True, remove_fc=True, show_params=False):
        super().__init__(make_layers(cfg[model]))
        self.ranges = ranges[model]

        if pretrained:
            exec("self.load_state_dict(models.%s(pretrained=True).state_dict())" % model)

        if not requires_grad:
            for param in super().parameters():
                param.requires_grad = False

        if remove_fc:  # delete redundant fully-connected layer params, can save memory
            del self.classifier

        if show_params:
            for name, param in self.named_parameters():
                print(name, param.size())

    def forward(self, x):
        output = {}

        # get the output of each maxpooling layer (5 maxpool in VGG net)
        for idx in range(len(self.ranges)):
            for layer in range(self.ranges[idx][0], self.ranges[idx][1]):
                x = self.features[layer](x)
            output["x%d"%(idx+1)] = x

        return output


ranges = {
    'vgg11': ((0, 3), (3, 6),  (6, 11),  (11, 16), (16, 21)),
    'vgg13': ((0, 5), (5, 10), (10, 15), (15, 20), (20, 25)),
    'vgg16': ((0, 5), (5, 10), (10, 17), (17, 24), (24, 31)),
    'vgg19': ((0, 5), (5, 10), (10, 19), (19, 28), (28, 37))
}

# cropped version from https://github.com/pytorch/vision/blob/master/torchvision/models/vgg.py
cfg = {
    'vgg11': [64, 'M', 128, 'M', 256, 256, 'M', 512, 512, 'M', 512, 512, 'M'],
    'vgg13': [64, 64, 'M', 128, 128, 'M', 256, 256, 'M', 512, 512, 'M', 512, 512, 'M'],
    'vgg16': [64, 64, 'M', 128, 128, 'M', 256, 256, 256, 'M', 512, 512, 512, 'M', 512, 512, 512, 'M'],
    'vgg19': [64, 64, 'M', 128, 128, 'M', 256, 256, 256, 256, 'M', 512, 512, 512, 512, 'M', 512, 512, 512, 512, 'M'],
}

def make_layers(cfg, batch_norm=False):
    layers = []
    in_channels = 3
    for v in cfg:
        if v == 'M':
            layers += [nn.MaxPool2d(kernel_size=2, stride=2)]
        else:
            conv2d = nn.Conv2d(in_channels, v, kernel_size=3, padding=1)
            if batch_norm:
                layers += [conv2d, nn.BatchNorm2d(v), nn.ReLU(inplace=True)]
            else:
                layers += [conv2d, nn.ReLU(inplace=True)]
            in_channels = v
    return nn.Sequential(*layers)
```

## 模型训练函数

```python
def train_FCN_model(train_dataset, val_dataset=None,
                    num_class=4, batch_size=4, num_workers=2, lr=1e-3, num_epochs=10):

    device = "cuda" if torch.cuda.is_available() else "cpu"

    train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True, num_workers=num_workers)
    val_loader = None
    if val_dataset is not None:
        val_loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False, num_workers=num_workers)

    vgg_model = VGGNet(pretrained=True, model='vgg16', requires_grad=True, remove_fc=True, show_params=False).to(device)
    fcn_model = FCNs(pretrained_net=vgg_model, n_class=num_class).to(device)
    fcn_model = nn.DataParallel(fcn_model)

    weights = torch.tensor([0.05, 0.8, 0.8, 0.8], device=device)
    criterion = nn.CrossEntropyLoss(weight=weights)
    optimizer = optim.SGD(fcn_model.parameters(), lr=lr, momentum=0.9)

    # -------- 从断点恢复部分 --------
    import os
    start_epoch = 0
    checkpoint_path = 'checkpoint_fcn.pth'

    if os.path.exists(checkpoint_path):
        print("🔁 检测到 checkpoint_fcn.pth，正在加载...")
        checkpoint = torch.load(checkpoint_path, map_location=device)
        fcn_model.load_state_dict(checkpoint['model_state_dict'])
        optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
        start_epoch = checkpoint['epoch']
        print(f"✅ 从第 {start_epoch} 个 epoch 继续训练。")

    # -------- 训练循环 --------
    for epoch in range(start_epoch, num_epochs):
        fcn_model.train()
        running_loss = 0.0

        for batch_idx, (images, masks) in enumerate(train_loader):
            images = images.to(device, dtype=torch.float)
            masks = masks.to(device, dtype=torch.long)

            optimizer.zero_grad()
            outputs = fcn_model(images)
            loss = criterion(outputs, masks)
            loss.backward()
            optimizer.step()
            running_loss += loss.item()

            if (batch_idx + 1) % 10 == 0:
                print(f"Epoch [{epoch+1}/{num_epochs}], Step [{batch_idx+1}/{len(train_loader)}], Loss: {loss.item():.4f}")

        avg_loss = running_loss / len(train_loader)
        print(f"Epoch [{epoch+1}/{num_epochs}] Average Loss: {avg_loss:.4f}")

        # -------- 验证集部分 --------
        if val_loader is not None:
            fcn_model.eval()
            val_loss = 0.0
            with torch.no_grad():
                for images, masks in val_loader:
                    images = images.to(device, dtype=torch.float)
                    masks = masks.to(device, dtype=torch.long)
                    outputs = fcn_model(images)
                    loss = criterion(outputs, masks)
                    val_loss += loss.item()
            print(f"Validation Loss: {val_loss/len(val_loader):.4f}")

        # -------- 保存checkpoint --------
        checkpoint = {
            'epoch': epoch + 1,
            'model_state_dict': fcn_model.state_dict(),
            'optimizer_state_dict': optimizer.state_dict(),
            'loss': avg_loss
        }
        torch.save(checkpoint, checkpoint_path)
        print(f"💾 模型已保存至 {checkpoint_path}")

    return fcn_model
```

## 模型训练

```python
fcn_model=train_FCN_model(train_dataset=train_dataset, val_dataset=val_dataset,
                    num_class=4, batch_size=2, num_workers=2, lr=1e-3, num_epochs=80)
```

## 测试集评估+可视化

```python
import torch
import os
import matplotlib.pyplot as plt
import numpy as np

def inference_FCN(fcn_model, test_dataset, device="cuda"):
    fcn_model.eval()
    fcn_model.to(device)

    results = []  # 存放预测 mask
    with torch.no_grad():
        for idx in range(len(test_dataset)):
            img_np, mask = test_dataset[idx]  
            if isinstance(img_np, np.ndarray):
                img_tensor = torch.from_numpy(img_np.transpose(2,0,1)).unsqueeze(0).float()
            else:
                img_tensor = img_np.unsqueeze(0).float()

            img_tensor = img_tensor.to(device)
            output = fcn_model(img_tensor)  # 输出 shape: [1, num_class, H, W]

            # 取每个像素最大概率的类别
            pred_mask = torch.argmax(output, dim=1).squeeze(0).cpu().numpy()
            results.append(pred_mask)

    return results

def visualize_with_true_and_pred(image_np, true_mask, pred_mask):
    """
    左图: 原图
    中图: 原图 + 真实mask轮廓
    右图: 原图 + 预测mask轮廓
    """
    plt.figure(figsize=(18,6))

    # 给每个类别分配显眼颜色
    colors = ['red', 'lime', 'blue', 'yellow', 'magenta', 'cyan', 'orange', 'purple']

    # 左图: 原图
    plt.subplot(1,3,1)
    plt.imshow(image_np)
    plt.title("Original Image")
    plt.axis("off")

    # 中图: 原图 + True Mask
    plt.subplot(1,3,2)
    plt.imshow(image_np)
    for cls in np.unique(true_mask):
        if cls == 0:  # 背景跳过
            continue
        mask = (true_mask == cls).astype(np.uint8)
        plt.contour(mask, colors=colors[cls % len(colors)], linewidths=2)
    plt.title("Image + True Mask")
    plt.axis("off")

    # 右图: 原图 + Predicted Mask
    plt.subplot(1,3,3)
    plt.imshow(image_np)
    for cls in np.unique(pred_mask):
        if cls == 0:
            continue
        mask = (pred_mask == cls).astype(np.uint8)
        plt.contour(mask, colors=colors[cls % len(colors)], linewidths=2)
    plt.title("Image + Predicted Mask")
    plt.axis("off")

    plt.show()


# ----------------- 使用示例 -----------------
device = "cuda" if torch.cuda.is_available() else "cpu"
checkpoint_path = 'checkpoint_fcn.pth'
if os.path.exists(checkpoint_path):
    print("存在预训练权重")
    #定义模型
    vgg_model = VGGNet(pretrained=True, model='vgg16', requires_grad=True, remove_fc=True, show_params=False).to(device)
    fcn_model = FCNs(pretrained_net=vgg_model, n_class=4).to(device)
    fcn_model = nn.DataParallel(fcn_model)
    # 加载保存的模型参数
    checkpoint_path = "/kaggle/working/checkpoint_fcn.pth"   # ← 改成你的路径
    checkpoint = torch.load(checkpoint_path, map_location=device)
    fcn_model.load_state_dict(checkpoint['model_state_dict'])

print(f"✅ 模型已成功加载，来自 epoch {checkpoint['epoch']}")

pred_masks = inference_FCN(fcn_model, test_dataset, device=device)

for i in range(5):
    img_np, true_mask = test_dataset[i]

    # 如果是 Tensor, 转为 numpy HWC
    if isinstance(img_np, torch.Tensor):
        img_np = img_np.permute(1,2,0).cpu().numpy()  # CHW -> HWC
        img_np = (img_np * 0.5 + 0.5)  # 如果 Dataset 有 Normalize
        img_np = np.clip(img_np, 0, 1)

    if isinstance(true_mask, torch.Tensor):
        true_mask = true_mask.cpu().numpy()

    visualize_with_true_and_pred(img_np, true_mask, pred_masks[i])
```
