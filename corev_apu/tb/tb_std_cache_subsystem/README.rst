================================================================================
Verifcation components for cache subsystem with coherency additions
================================================================================

This folder contains verification components for verifying the
**std_cache_subsystem** module with coherency additions.


--------------------------------------------------------------------------------
Components
--------------------------------------------------------------------------------

cva6 (cache_dummy)
================================================================================
This is a version of the **cva6** core with only the cache subsystem remaining.
A test bench verifying the caches must drive the cache request ports.


std_cache_scoreboard
================================================================================
This scoreboard checks the behaviour of a single cache subsystem. The scoreboard
receives transactions via a number of mailboxes and verifies that the received
transactions match what is expected. The scoreboard maintains an internal model
of the cache, :code:`cache_status`, to be able to predict transactions and
verify against the real memory.

The scoreboard is activated by the task :code:`run()` which in turn starts a
number of internal tasks that receives transactions.

* Cache load and store requests are received by :code:`get_cache_msg()`, which
  then calls the :code:`check_cache_msg()` task.
* Snoop transactions are received by the :code:`check_snoop()` task.
* AMO requests are received by the :code:`check_amo_msg()` task.
* Management requests are received by the :code:`check_mgmt_trans()` task.


check_cache_msg()
--------------------------------------------------------------------------------
This task receives dcache load and store requests and takes the following
actions:

#. The address is checked to determine which memory region the request targets.
#. For requests to a cached region, the cache model is queried to expect a hit
   or miss.
#. The expected AXI transactions are collected from the AXI monitors. If the
   received transactions don't match what is expected, or if the expected
   transaction is not received before a timeout limit, an error is raised.
#. Under some circumstances, changes in the cache caused by other transactions
   (such as a :code:`ClearInvalid` request to the snoop controller) causes the
   hit or miss subroutines to be run again to match the behaviour of the cache
   controller.
#. The message is passed on to the :code:`update_cache_from_req` task.

The check_cache_msg task probes and waits for some internal signals via the
:code:`gnt_vif` interface to be able to get a precise timing of the expected
transactions.


check_snoop()
--------------------------------------------------------------------------------
This task receives snoop requests and takes the following actions:

#. Check if the request targets a cacheable region. If so, sends the request to
   the :code:`update_cache_from_snoop()` task.
#. Waits for the snoop response and compares it with an expected response.


update_cache_from_req(), update_cache_from_snoop()
--------------------------------------------------------------------------------
These tasks receive dcache and snoop requests requests respectively and take the
following actions:

#. Wait for cache access to be granted by probing internal signals.
#. Update the internal cache model based on current state of the cache and the
   received request.
#. Check that the contents of the internal cache model matches the cache in the
   DUT.

When checking the cache contents for a certain entry in the cache, all other
ways for the same index are also checked against the DUT. This is a trade-off
between a minimal check (only check the value of the expected way) and a maximal
check (check the contents of the entire cache on every access).


check_amo_msg()
--------------------------------------------------------------------------------
This task receives dcache AMO requests and takes the following actions:

#. The targeted cache entry is invalidated if it is a hit. If the cache entry is
   dirty, a write back transaction is expected.
#. The address is checked to determine which memory region the request targets.
#. The expected AXI transactions are collected. If the received transactions
   don't match what is expected, or if no transaction is received before a
   timeout limit, an error is raised.


check_mgmt_trans()
--------------------------------------------------------------------------------
This task receives dcache management requests. Currently only :code:`flush` is
supported.

When a flush request is received, the following actions are taken:

#. The cache model is traversed. For each entry:
    #. If the entry is dirty, a write-back transaction is expected.
    #. If the entry is valid, wait for cache access to be granted (internal
       signal probed).
    #. Clear the cache entry.


std_dcache_checker
================================================================================

This scoreboard checks cache behaviour on the system level, i.e. possibly
involving more than one cache subsystem. The scoreboard has two main tasks,
:code:`mon_dcache()`, and :code:`check_amo_lock()`.

.. attention::
   TODO: change name of std_cache_checker to something better, e.g.
   :code:`std_dcache_system_scoreboard`.


mon_dcache()
--------------------------------------------------------------------------------
This task verifies that the contents of the data caches in the systems match,
and that they also match with the contents of the main memory.

Upon each write to a data cache, the contents of all ways in the written set is
checked against the corresponding ways in other data caches in the system:

- If multiple entries are valid and have matching tags, then

  - verify that the data also matches.

  - verify that all entries are marked as shared.

  - verify that at most one of the entries is marked as dirty.

- If no entry for a given tag is marked dirty, then

  - verify that the data matches the data in main memory.

The check against the main memory can be disabled by setting the internal
variable :code:`enable_mem_check` to 0, e.g. in case of a LLC present in the
system.


check_amo_lock()
--------------------------------------------------------------------------------
This task is only enabled in specific test cases. It verifies a lock mechanism
using :code:`AMO_SWAP` to set (lock) and clear (unlock) a data entry.

The task keeps a local record of locked addresses. It flags an error if any of
the following occurs:

- A lock request succeeds to an address that is already locked.
- An unlock request succeeds to an address that is not locked or is locked by
  another core.
- An unlock request fails.

Note that this task can only be used in specific directed tests that implement
correct software lock mechanisms. It is disabled by default.


amo_driver
================================================================================
The AMO driver generates AMO requests by calling the :code:`req()` or
:code:`req_resp()` tasks. :code:`req_resp()` will return the response and
optionally compare it with an expected result.


amo_monitor
================================================================================
The AMO monitor detects AMO requests and responses and puts them in the
:code:`req_mbox` and :code:`resp_mbox` mailboxes, respectively. The monitor is
started through the :code:`monitor()` task.


dcache_driver
================================================================================
The dcache driver generates dcache write and read requests by calling the
:code:`wr()`, :code:`rd()`, or :code:`rd_resp()` tasks. :code:`rd_resp()` will
return the read response and optionally compare it with an expected result.

The requests can use specified address, data, etc, or use random values.

The read requests can be configured to wait for a result before finishing. The
default is to allow a new read request to be submitted while another one is
ongoing.

The driver can emulate killing requests by setting the :code:`kill` argument.
Then the read request will be killed immediately after receiving a read grant.
It is also possible to randomize the possibility of a read request being killed
using the :code:`rand_kill` argument.


dcache_monitor
================================================================================
The dcache monitor detects dcache write and read requests and responses and puts
them in the :code:`req_mbox` and :code:`resp_mbox` mailboxes, respectively. The
monitor is started through the :code:`monitor()` task.


icache_driver
================================================================================
The icache driver generates icache read requests by calling the :code:`rd()` or
:code:`rd_resp()` tasks. The :code:`rd_resp()` tasks will wait for a result
before finishing while :code:`rd_resp()` task allows a new read request to be
submitted while another one is ongoing.

The :code:`start_random_req()` task will continuosly generate bursts of
requests.

.. note:: There is currently no icache monitor implemented.


dcache_mgmt_driver
================================================================================
The dcache mgmt driver generates management transactions to the dcache.
Currently only the :code:`flush()` task is implemented, generating a flush
request.


dcache_mgmt_monitor
================================================================================
The dcache mgmt monitor detects management transactions to the dcache adn puts
them in the :code:`mbox` mailbox. Currently only flush transactions are
detected.
