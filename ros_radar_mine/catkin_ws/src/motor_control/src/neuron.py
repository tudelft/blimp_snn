import torch
import torch.nn as nn
from torch.nn.parameter import Parameter

from utils import _set_no_grad
import functional as sf


#########################################################
# Input Neuron
#########################################################
class BaseInput(nn.Module):
    r"""Simple feed-through layer of neurons used for generating a trace.
    
    :param cells_shape: a list or tuple that specifies the shape of the neurons in the conventional PyTorch format, but with the batch size as the first dimension.
    :param dt: duration of a single timestep.
    """

    def __init__(self, cells_shape, dt):
        super(BaseInput, self).__init__()
        self.register_buffer("trace", torch.zeros(*cells_shape, dtype=torch.float))
        self.register_buffer("dt", torch.tensor(dt, dtype=torch.float))
        self.register_buffer("spikes", torch.zeros(*cells_shape, dtype=torch.bool))

    def reset_state(self):
        r"""Reset cell states that accumulate over time during simulation."""
        self.trace.fill_(0)
        self.spikes.fill_(False)

    def no_grad(self):
        r"""Turn off gradient storing."""
        _set_no_grad(self)

    def init_neuron(self):
        r"""Initialize state, and turn off gradients."""
        self.no_grad()
        self.reset_state()

    def convert_input(self, x):
        r"""Convert torch.bool input to the datatype set for arithmetics.
        
        :param x: Input Tensor of torch.bool type.
        """
        return x.type(self.trace.dtype)

    def forward(self, x):
        raise NotImplementedError("Input neurons must implement `forward`")

    def update_trace(self, x):
        r"""Placeholder for trace update function."""
        raise NotImplementedError("Input neurons must implement `update_trace`")


class Input(BaseInput):
    r"""Standard input neuron, used to propagate input traces to the following :class:`Connection` object, and calculates a trace.
    
    :param cells_shape: a list or tuple that specifies the shape of the neurons in the conventional PyTorch format, but with the batch size as the first dimension.
    :param dt: duration of a single timestep.
    :param alpha_t: scaling constant for the increase of the trace by a single spike.
    :param tau_t: decay parameter for the trace.
    :param update_type: string, either ``'linear'`` or ``'exponential'``, default is ``'linear'``.
    """

    def __init__(self, cells_shape, dt, alpha_t, tau_t, update_type="linear"):
        super(Input, self).__init__(cells_shape, dt)

        # Fixed parameters
        self.register_buffer(
            "alpha_t", alpha_t * torch.ones(cells_shape, dtype=torch.float)
        )
        self.register_buffer(
            "tau_t", tau_t * torch.ones(cells_shape, dtype=torch.float)
        )

        # Type of updates
        if update_type == "linear":
            assert (
                (self.tau_t >= 0.0) & (self.tau_t <= 1.0)
            ).all(), "Decays for linear updates should be in the interval [0, 1]."
            self.trace_update = sf.linear_trace_update
        elif update_type == "exponential":
            self.trace_update = sf.exponential_trace_update
        else:
            raise ValueError("Unsupported trace type")

        self.init_neuron()

    def update_trace(self, x):
        r"""Converts input spikes and updates the trace.
        
        :param x: Tensor with the input spikes.
        """
        x = self.convert_input(x)
        self.trace = self.trace_update(self.trace, x, self.alpha_t, self.tau_t, self.dt)

    def forward(self, x):
        r"""Propagate spikes through input neurons and compute trace.
        
        :param x: Input spikes
        """
        self.update_trace(x)
        self.spikes.copy_(x)
        return x, self.trace


#########################################################
# Base Neuron
#########################################################
class BaseNeuron(nn.Module):
    r"""Base neuron model, is a container to define basic neuron functionalties.

    Defines basic spiking, voltage and trace characteristics. Just has to
    adhere to the API functionalities to integrate within Connection modules.

    Make sure the Neuron class receives input voltage for each neuron and
    returns a Tensor indicating which neurons have spiked.

    :param cells_shape: a list or tuple that specifies the shape of the neurons in the conventional PyTorch format, but with the batch size as the first dimension.
    :param thresh: spiking threshold, when the cells' voltage surpasses this value it generates a spike.
    :param v_rest: voltage resting value, the :class:`Neuron` will default back to this over time or after spiking.
    :param dt: duration of a single timestep.
    :param duration_refrac: Number of timesteps the :class:`Neuron` is dormant after spiking. Make sure ``dt`` fits an integer number of times in ``duration refrac``.
    :param update_type: string, either ``'linear'`` or ``'exponential'``, default is ``'linear'``.
    :param store_trace: ``Boolean`` flag to store the complete spiking history, defaults to ``False``.
    """

    def __init__(
        self, cells_shape, thresh, v_rest, dt, duration_refrac, store_trace=False
    ):
        super(BaseNeuron, self).__init__()

        # Check compatibility of dt and refrac counting
        assert (
            duration_refrac % dt == 0
        ), "dt does not fit an integer amount of times in duration_refrac."
        assert duration_refrac >= 0, "duration_refrac should be non-negative."

        # Fixed parameters
        self.register_buffer("v_rest", torch.tensor(v_rest, dtype=torch.float))
        self.register_buffer("dt", torch.tensor(dt, dtype=torch.float))
        self.register_buffer(
            "duration_refrac", torch.tensor(duration_refrac, dtype=torch.float)
        )
        self.register_buffer("thresh_center", torch.tensor(thresh, dtype=torch.float))

        # Define dynamic parameters
        self.register_buffer("spikes", torch.empty(*cells_shape, dtype=torch.bool))
        self.register_buffer("v_cell", torch.empty(*cells_shape, dtype=torch.float))
        self.register_buffer("trace", torch.empty(*cells_shape, dtype=torch.float))
        self.register_buffer(
            "refrac_counts", torch.empty(*cells_shape, dtype=torch.float)
        )

        # Define learnable parameters
        self.thresh = Parameter(
            torch.empty(*cells_shape, dtype=torch.float), requires_grad=False
        )

        # In case of storing a complete, local copy of the activity of a neuron
        #if store_trace:
        #    complete_trace = torch.zeros(*cells_shape, 1, dtype=torch.bool)
        #else:
        complete_trace = None
        self.register_buffer("complete_trace", complete_trace)

    def spiking(self):
        r"""Return cells that are in spiking state."""
        self.spikes.copy_(self.v_cell >= self.thresh)
        return self.spikes.clone()

    def refrac(self, spikes):
        r"""Basic counting version of cell refractory period.

        Can be overwritten in case of the need of more refined functionality.
        """
        if self.duration_refrac > 0:
            self.refrac_counts[self.refrac_counts > 0] -= self.dt
            self.refrac_counts += self.duration_refrac * self.convert_spikes(spikes)
        self.v_cell.masked_fill_(spikes, self.v_rest)

    def concat_trace(self, x):
        r"""Concatenate most recent timestep to the trace storage."""
        self.complete_trace = torch.cat([self.complete_trace, x.unsqueeze(-1)], dim=-1)

    def fold(self, x):
        r"""Fold incoming spike train by summing last dimension."""
        if isinstance(x, (list, tuple)):
            x = torch.cat(x, dim=-1)
        return x.sum(-1)

    def unfold(self, x):
        r"""Move the last dimension (all incoming to single neuron in current layer) to first dim.

        This is done because PyTorch broadcasting does not support broadcasting over the last dim.
        """
        shape = x.shape
        return x.view(shape[-1], *shape[:-1])

    def convert_spikes(self, spikes):
        r"""Cast ``torch.bool`` spikes to datatype that is used for voltage and weights"""
        return spikes.to(self.v_cell.dtype)

    def reset_state(self):
        r"""Reset cell states that accumulate over time during simulation."""
        self.v_cell.fill_(self.v_rest)
        self.spikes.fill_(False)
        self.refrac_counts.fill_(0)
        self.trace.fill_(0)
        #if self.complete_trace is not None:
        #    self.complete_trace = torch.zeros(
        #        *self.v_cell.shape, 1, device=self.v_cell.device
        #    ).bool()

    def reset_thresh(self):
        r"""Reset threshold to initialization values, allows for different standard thresholds per neuron."""
        self.thresh.copy_(torch.ones_like(self.thresh) * self.thresh_center)

    def no_grad(self):
        r"""Turn off learning and gradient storing."""
        _set_no_grad(self)

    def init_neuron(self):
        r"""Initialize state, parameters, and turn off gradients."""
        self.no_grad()
        self.reset_state()
        self.reset_thresh()

    def forward(self, x):
        raise NotImplementedError("Neurons must implement `forward`")

    def update_trace(self, x):
        r"""Placeholder for trace update function."""
        raise NotImplementedError("Neurons must implement `update_trace`")

    def update_voltage(self, x):
        r"""Placeholder for voltage update function."""
        raise NotImplementedError("Neurons must implement `update_voltage`")


#########################################################
# IF Neuron
#########################################################
class IFNeuron(BaseNeuron):
    r"""Basic integrate and fire neuron, cell voltage does not decay over time.

    :param cells_shape: a list or tuple that specifies the shape of the neurons in the conventional PyTorch format, but with the batch size as the first dimension.
    :param thresh: spiking threshold, when the cells' voltage surpasses this value it generates a spike.
    :param v_rest: voltage resting value, the :class:`Neuron` will default back to this over time or after spiking.
    :param alpha_v: scaling constant for the increase of the voltage by a single spike.
    :param alpha_t: scaling constant for the increase of the trace by a single spike.
    :param dt: duration of a single timestep.
    :param duration_refrac: Number of timesteps the :class:`Neuron` is dormant after spiking. Make sure ``dt`` fits an integer number of times in ``duration refrac``.
    :param tau_t: decay parameter for the trace.
    :param update_type: string, either ``'linear'`` or ``'exponential'``, default is ``'linear'``.
    :param store_trace: ``Boolean`` flag to store the complete spiking history, defaults to ``False``.
    """

    def __init__(
        self,
        cells_shape,
        thresh,
        v_rest,
        alpha_v,
        alpha_t,
        dt,
        duration_refrac,
        tau_t,
        update_type="linear",
        store_trace=False,
    ):
        super(IFNeuron, self).__init__(
            cells_shape, thresh, v_rest, dt, duration_refrac, store_trace=store_trace
        )

        # Fixed parameters
        self.register_buffer(
            "alpha_v", alpha_v * torch.ones(cells_shape, dtype=torch.float)
        )
        self.register_buffer(
            "alpha_t", alpha_t * torch.ones(cells_shape, dtype=torch.float)
        )
        self.register_buffer(
            "tau_t", tau_t * torch.ones(cells_shape, dtype=torch.float)
        )

        # Type of updates
        if update_type == "linear":
            assert (
                (self.tau_t >= 0.0) & (self.tau_t <= 1.0)
            ).all(), "Decays for linear updates should be in the interval [0, 1]."
            self.trace_update = sf.linear_trace_update
        elif update_type == "exponential":
            self.trace_update = sf.exponential_trace_update
        else:
            raise ValueError("Unsupported trace type")

        self.init_neuron()

    def update_trace(self, x):
        r"""
        :param x: Incoming/presynaptic spikes
        """
        spikes = self.convert_spikes(x)
        self.trace = self.trace_update(
            self.trace, spikes, self.alpha_t, self.tau_t, self.dt
        )

    def update_voltage(self, x):
        r"""
        :param x: Incoming/presynaptic spikes
        """
        self.v_cell = sf.if_voltage_update(
            self.v_cell, x, self.alpha_v, self.refrac_counts
        )

    def forward(self, x):
        r"""
        :param x: Incoming/presynaptic spikes

        :return: Neuron output spikes and trace
        """
        x = self.fold(x)
        self.update_voltage(x)
        spikes = self.spiking()
        self.update_trace(spikes)
        self.refrac(spikes)
        if self.complete_trace is not None:
            self.concat_trace(spikes)
        return spikes, self.trace


#########################################################
# LIF Neuron
#########################################################
class LIFNeuron(BaseNeuron):
    r"""Leaky integrate and fire neuron, cell voltage decays over time.
    
    :param cells_shape: a list or tuple that specifies the shape of the neurons in the conventional PyTorch format, but with the batch size as the first dimension.
    :param thresh: spiking threshold, when the cells' voltage surpasses this value it generates a spike.
    :param v_rest: voltage resting value, the :class:`Neuron` will default back to this over time or after spiking.
    :param alpha_v: scaling constant for the increase of the voltage by a single spike.
    :param alpha_t: scaling constant for the increase of the trace by a single spike.
    :param dt: duration of a single timestep.
    :param duration_refrac: Number of timesteps the :class:`Neuron` is dormant after spiking. Make sure ``dt`` fits an integer number of times in ``duration refrac``.
    :param tau_v: decay parameter for the voltage.
    :param tau_t: decay parameter for the trace.
    :param update_type: string, either ``'linear'`` or ``'exponential'``, default is ``'linear'``.
    :param store_trace: ``Boolean`` flag to store the complete spiking history, defaults to ``False``.
    """

    def __init__(
        self,
        cells_shape,
        thresh,
        v_rest,
        alpha_v,
        alpha_t,
        dt,
        duration_refrac,  # From here on class specific params
        tau_v,
        tau_t,
        update_type="linear",
        store_trace=False,
    ):
        super(LIFNeuron, self).__init__(
            cells_shape, thresh, v_rest, dt, duration_refrac, store_trace=store_trace
        )

        # Fixed parameters
        self.register_buffer(
            "alpha_v", alpha_v * torch.ones(cells_shape, dtype=torch.float)
        )
        self.register_buffer(
            "alpha_t", alpha_t * torch.ones(cells_shape, dtype=torch.float)
        )
        self.register_buffer(
            "tau_v", tau_v * torch.ones(cells_shape, dtype=torch.float)
        )
        self.register_buffer(
            "tau_t", tau_t * torch.ones(cells_shape, dtype=torch.float)
        )

        # Type of updates
        if update_type == "linear":
            assert (
                (self.tau_v >= 0.0)
                & (self.tau_v <= 1.0)
                & (self.tau_t >= 0.0)
                & (self.tau_t <= 1.0)
            ).all(), "Decays for linear updates should be in the interval [0, 1]."
            self.voltage_update = sf.lif_linear_voltage_update
            self.trace_update = sf.linear_trace_update
        elif update_type == "exponential":
            self.voltage_update = sf.lif_exponential_voltage_update
            self.trace_update = sf.exponential_trace_update
        else:
            raise ValueError("Unsupported update type")

        self.init_neuron()

    def update_trace(self, x):
        r"""
        :param x: Incoming/presynaptic spikes
        """
        spikes = self.convert_spikes(x)
        self.trace = self.trace_update(
            self.trace, spikes, self.alpha_t, self.tau_t, self.dt
        )

    def update_voltage(self, x):
        r"""
        :param x: Incoming/presynaptic spikes
        """
        self.v_cell = self.voltage_update(
            self.v_cell,
            self.v_rest,
            x,
            self.alpha_v,
            self.tau_v,
            self.dt,
            self.refrac_counts,
        )

    def forward(self, x):
        r"""
        :param x: Incoming/presynaptic spikes

        :return: Neuron output spikes and trace
        """
        x = self.fold(x)
        self.update_voltage(x)
        spikes = self.spiking()
        self.update_trace(spikes)
        self.refrac(spikes)
        if self.complete_trace is not None:
            self.concat_trace(spikes)
        return spikes, self.trace