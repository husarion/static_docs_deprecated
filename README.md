# How to edit Husarion Docs

## Basics

This repository contains sources of the documentation that is available as [Tutorials](https://husarion.com/core2/tutorials/) and [Manuals](https://husarion.com/core2/manuals/core2/) pages. They are automatically generated from master branch, but before that, the pull reuqests has to be approved by admin. The preview on GitHub can look slightly different than the publicated version.

The sources are written using Markdown. The syntax of Markdown is well-described on the [Daring Fireball website](https://daringfireball.net/projects/markdown/syntax). You can also use HTML if Markdown is not enough.

Source text for articles is stored in **/src** catalog.

## Images

Images are stored in **/assets/img**

Standard Markdown syntax is applicable for images, but there is an additional possibility to create galleries:

 ```
<div class="gallery gallery-6">
    ![The caption under image](/assets/img/image.png "the description shown after enlarging the image")
    ![The caption under image 2](/assets/img/image2.png "the description shown after enlarging the image 2")
</div>
 ```
The `gallery-6` means 6 images in row. If necessary, also the `gallery-2`, `gallery-3` etc. can be implemented.

## Videos

From Vimeo:
```
<div align="center">
<iframe src="https://player.vimeo.com/video/225576807" width="427" height="240" frameborder="0" webkitallowfullscreen mozallowfullscreen allowfullscreen></iframe>
</div>
```

From Youtube:
```
<iframe width="1156" height="560" src="https://www.youtube.com/embed/JkIj5ssHpKw" frameborder="0" gesture="media" allowfullscreen></iframe>
```

## Metadata headers and ToC generation

Each .md file contains the metadata needed for generate HTML pages that are properly referenced to ToC on the left menu, here is an example:

``` 
---
title: 'The first step of tutorial'		// page title, not critical
platform: 'CORE2'				// which hardware platform is documented
autotoc: true                			// generate ToC or not?
layout: layout.hbs           			// don't change
order: 2                     			// the order in section (and the number displayed in ToC)
---
```

The ToC has 3 or 4 levels. It depends on the page.

For Tutorials:
<table>
    <tr>
       <th>Level</th>
       <th>ToC level</th>
       <th>Syntax</th>
    </tr>
    <tr>
        <td>1</td>
        <td>1. Article</td>
        <td># Article # - must be consistent with filename (article.md)*</td>
    </tr>
	<tr>
        <td>2</td>
        <td>1.1. Chapter</td>
        <td>## Chapter ##</td>
    </tr>
	<tr>
        <td>3</td>
        <td> Subchapter </td>
        <td>### Subchapter ###</td>
    </tr>
</table>

For Manuals:
<table>
    <tr>
       <th>Level</th>
       <th>ToC level</th>
       <th>Syntax</th>
    </tr>
    <tr>
        <td>1</td>
        <td><strong>Section</strong></td>
        <td># Section # - must be consistent with filename (section.md)*</td>
    </tr>
	<tr>
        <td>2</td>
        <td>1. Article</td>
        <td># Article #</td>
    </tr>
	<tr>		
        <td>3</td>
        <td>1.1. Chapter</td>
        <td>## Chapter ##</td>
    </tr>
	<tr>
        <td>4</td>
        <td> Subchapter </td>
        <td>### Subchapter ###</td>
    </tr>
</table>

* Only small letters and hyphens are allowed in filename. If the article name is \# Tutorial for CORE2 \#, the filename has only small letters and hyphens instead whitespaces: `tutorial-for-core2.md`.
