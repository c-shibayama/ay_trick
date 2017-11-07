ay_trick
==================
Environment for robot system development and experiments.  `ay_trick` provides a CUI tool where you can interact with robots.  Once the initialization including ROS communication setup is done, you do not have to do it again.  You can run external Python scripts.

`ay_trick` is implemented with Python.


Author
==================
Akihiko Yamaguchi, http://akihikoy.net/


Requirements
==================
See `manifest.xml`.


Directories
==================

scripts
----------------------------
Main Python tools.

scripts/lib
----------------------------
**Library scripts** that are executed from the CUI tool.

models
----------------------------
Model files such as collision and geometry models of objects.


Usage
==================

cui_tool
---------------------------
CUI tool.
Execute it from the root directory of `ay_trick`.

```
$ scripts/cui_tool.py
```

Note: roscore should be working beforehand.
This program provides a CUI to execute library scripts stored in `scripts/lib`.  This is useful since we can execute many programs without ROS reconnection to robots.

If robotic environment is not setup, it will just display:

```
trick or quit>
```

Type `test.test` and press the enter:

```
trick or quit> test.test
<core_tool.TCoreTool object at 0x7f87ff8320d0>
Hello world!
Test.
  Usage: test [FLOAT_VALUE]
```

It executed `scripts/lib/test/test.py` (`test.` specifies a sub directory).

If a robot is ready (of course works with simulated robots too), we can run robot control scripts.  For example,

```
Baxter:trick or quit|L> j
```

Use `-help` option to see a brief explanation of each script; e.g.

```
trick or quit> test.test -help
Test.
  Usage: test [FLOAT_VALUE]
```


Troubles
==================
Send e-mails to the author.
