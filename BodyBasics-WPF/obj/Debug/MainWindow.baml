   M S B A M L   `   `   `  ����  BodyBasics-WPF4    .Microsoft.Samples.Kinect.BodyBasics.MainWindow   5   	   Ddxlc;http://schemas.devexpress.com/winfx/2008/xaml/layoutcontrol  R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35\ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35D 9http://schemas.microsoft.com/winfx/2006/xaml/presentation    5   	   R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     5   	   d `Microsoft.Kinect.Wpf.Controls, Version=2.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e350k(http://schemas.microsoft.com/kinect/2014  5   	   ?telerik1http://schemas.telerik.com/2008/xaml/presentation  5   	   -   5          � WindowState$  	Maximized=�5   &    � WindowStyle$
 None=�6>    � 	Resources 5         CategoryStyle �� 
TargetType �� Property �� Value �� 
TargetType �� CornerRadius �� Padding  	ItemStyle  ItemSelectedStyle��%�  &	        &	     &	 �    �� 5   
   " ��6&   .�5	      ��  � �� 5
      �� 5      " ��6&   .R�5      �� $ 15��6!   $��5q�63   
�������
�������$��	0,25,0,25q�6o   $ 10,10,10,10q�6�   ._�5      �� 5      5      5      5      5      �� 5   
   " ��6"   .�5      ��  � �� 5      �� 5      " ��6&   .R�5      �� 
�������
�������$��	25,0,25,0q�6I   $ 10,10,10,10q�6\   ._�5      �� 5      5      5      5!      5"      �� 5$   
   " ��6*   .�5%      ��  � �� 5&      �� 5'      " ��6&   .R�5(      �� 
�������
���  ��$��	25,0,25,0q�6G   $ 10,10,10,10q�6Z   ._�5)      �� 5+      5,      5-      5/      50      51      .��52      0  *Microsoft.Kinect.Wpf.Controls.KinectRegion  -   5        	  Name$	 kinectRegion��52      .��53   
   � .+�54      �� $
��None=�63   ��6   �� *imageSource+63   �� 56      $��Hidden=�6   $��Disabled=�6B   ��. 
   IsScrollInertiaEnabled
 .  $
��1096��6�   $��Right=�6�    K� ZIndex$
 1000��6�   .��57      �� -   5         ��Name$ Panel��57      $��Center=�6F   ��6.   �� ��58      �� 59      .R�5:      �  � Orientation$ 
Horizontal=�6(   $��
75,50,75,0q�6A   5;      ��5=      �� 5>      .R�5?      �� .+�5@      �� -   5        #��[ 5@   %   .��5A   "   �� $	��150��6B   $	��150��6N   ��6(   �� *Field1+6N   5B      �� 5C   "   
���   �
�������$��	25,0,25,0q�6j    �� Padding$ 5,5,5,5q�6}   $��Center=�6�    �� TextAlignment$ Center=�6�   $
��12pt(�6�   $	��170��6�   ��6,   �� *Field2+6�   5D      5E      5G      5H      �� 5J      $��Disabled=�6   $��Hidden=�6D   ��. 
 .  $
��Left=�6�   $	��230��6�   .��5K      �� -   5        $ Panel2��5K      ���	#FF007BF5��5L      �� 5M      .R�5N      � $��Center=�6(   $��Center=�6C   $ Vertical=�6`   $��10,10,10,10q�6w   5O      .@�5Q      �� -   5         ��Name$ btnNecklace��5Q      #��[  62   .��5R      �� $��Images\Necklace\1.png��6    $	��100��6?   $	��100��6K   5S      �� 5T      -   5        $ btnLongNecklace��5T      #��[  66   .��5U      �� $ ��Images\Long Necklace\1.png��6    $	��100��6D   $	��100��6P   5V      �� 5W      -   5        $ btnDoubleNecklace��5W      #��[  68   .��5X      �� $"��Images\Double Necklace\1.png��6    $	��100��6F   $	��100��6R   5Y      �� 5Z      -	   5        $ btnOttiyanam��5Z      #��[  63   .��5[      �� $��Images\Ottiyanam\1.png��6    $	��100��6@   $	��100��6L   5\      5]      5^      �� 5`      .+�5a      �� -
   5        $��imgNecklace��5a      �� 5b      -   5        $��imgLongNecklace��5b      �� 5c      -   5        $��imgDoubleNecklace��5c      �� 5d      -   5        $��imgOttiyanam��5d      5e      5f      5g      5h      